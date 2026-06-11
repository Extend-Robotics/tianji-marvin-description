// Robot.cpp - Linux 版本
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <time.h>
#include <signal.h>
#include "marvin_ros_control/FX_Robot.h"
#include "stdio.h"
#include "stdlib.h"

// // Linux 网络初始化（无操作）
// static inline int socket_init() {
//     return 0; // Linux 不需要显式初始化
// }

// Linux 定时器回调函数类型
typedef void (*TimerCallbackFunc)(UINT, UINT, DWORD_PTR, DWORD_PTR, DWORD_PTR);

// 定时器信息结构
typedef struct {
    TimerCallbackFunc callback;
    DWORD_PTR user;
    UINT interval;  // 间隔时间（毫秒）
    int running;
    pthread_t thread;
    int timer_id;
} TimerContext;

// 全局定时器列表
static TimerContext* g_timers[100] = {0};
static int g_timer_count = 0;
static pthread_mutex_t g_timer_mutex = PTHREAD_MUTEX_INITIALIZER;

// 定时器线程函数
static void* timer_thread(void* arg) {
    TimerContext* ctx = (TimerContext*)arg;
    
    while (ctx->running) {
        usleep(ctx->interval * 1000);  // 转换为微秒
        
        if (ctx->running && ctx->callback) {
            ctx->callback(0, 0, ctx->user, 0, 0);
        }
    }
    
    // 清理
    pthread_mutex_lock(&g_timer_mutex);
    for (int i = 0; i < g_timer_count; i++) {
        if (g_timers[i] == ctx) {
            g_timers[i] = NULL;
            break;
        }
    }
    pthread_mutex_unlock(&g_timer_mutex);
    
    free(ctx);
    return NULL;
}

// timeSetEvent 的 Linux 实现
MMRESULT timeSetEvent(UINT uDelay, UINT uResolution, 
                     TimerCallbackFunc lpTimeProc, DWORD_PTR dwUser, 
                     UINT fuEvent) {
    (void)uResolution;  // Linux 不使用分辨率参数
    (void)fuEvent;      // 简化事件类型处理
    
    TimerContext* ctx = (TimerContext*)malloc(sizeof(TimerContext));
    if (!ctx) return 0;
    
    ctx->callback = lpTimeProc;
    ctx->user = dwUser;
    ctx->interval = uDelay;
    ctx->running = 1;
    ctx->timer_id = ++g_timer_count;
    
    // 创建定时器线程
    if (pthread_create(&ctx->thread, NULL, timer_thread, ctx) != 0) {
        free(ctx);
        return 0;
    }
    
    // 设置线程分离
    pthread_detach(ctx->thread);
    
    // 保存定时器上下文
    pthread_mutex_lock(&g_timer_mutex);
    if (g_timer_count < 100) {
        g_timers[g_timer_count-1] = ctx;
    }
    pthread_mutex_unlock(&g_timer_mutex);
    
    return ctx->timer_id;
}

// timeKillEvent 的 Linux 实现
MMRESULT timeKillEvent(MMRESULT uTimerID) {
    pthread_mutex_lock(&g_timer_mutex);
    
    for (int i = 0; i < g_timer_count; i++) {
        if (g_timers[i] && g_timers[i]->timer_id == uTimerID) {
            g_timers[i]->running = 0;
            // 注意：我们不会在这里 join 线程，因为它是分离的
            pthread_mutex_unlock(&g_timer_mutex);
            return 0;  // 成功
        }
    }
    
    pthread_mutex_unlock(&g_timer_mutex);
    return 1;  // 失败：未找到定时器
}

// 定义 CALLBACK 宏
#ifndef CALLBACK
#define CALLBACK
#endif

// 定义常量
#define TIME_PERIODIC 1

// 静态变量
static CRobot* m_InsRobot = NULL;

// CMarvNetAgent 实现
CMarvNetAgent::CMarvNetAgent() {
    m_nc._local_sock = INVALID_SOCKET;
    m_nc._tosock_ = INVALID_SOCKET;
    m_nc._toLen = 0;
    m_nc._localLen = 0;
    m_nc._from_valid = 0;
    memset(&m_nc._local, 0, sizeof(m_nc._local));
    memset(&m_nc._to, 0, sizeof(m_nc._to));

    m_buf.m_Slen = 0;
    m_buf.m_Rlen = 0;
    m_buf.m_STag = 0;

    m_LinkTag = FX_FALSE;
}

CMarvNetAgent::~CMarvNetAgent() {
    if (m_nc._local_sock != INVALID_SOCKET) {
        closesocket(m_nc._local_sock);
        m_nc._local_sock = INVALID_SOCKET;
    }
    if (m_nc._tosock_ != INVALID_SOCKET) {
        closesocket(m_nc._tosock_);
        m_nc._tosock_ = INVALID_SOCKET;
    }
}

FX_BOOL CMarvNetAgent::OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4, FX_INT32L port) {
    if (m_LinkTag == FX_TRUE) {
        return FX_FALSE;
    }

    // Linux 不需要 WSAStartup
    // 初始化 socket
    memset(&m_nc._local, 0, sizeof(m_nc._local));
    m_nc._localLen = sizeof(struct sockaddr_in);
    m_nc._local.sin_family = AF_INET;
    m_nc._local.sin_port = htons(port);
    m_nc._local.sin_addr.s_addr = INADDR_ANY;
    
    m_nc._local_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_nc._local_sock == INVALID_SOCKET) {
        std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
        return FX_FALSE;
    }

    // 设置非阻塞模式 (Linux 方式)
    int flags = fcntl(m_nc._local_sock, F_GETFL, 0);
    if (fcntl(m_nc._local_sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        std::cerr << "Failed to set non-blocking: " << strerror(errno) << std::endl;
        closesocket(m_nc._local_sock);
        m_nc._local_sock = INVALID_SOCKET;
        return FX_FALSE;
    }

    // 绑定 socket
    if (bind(m_nc._local_sock, (struct sockaddr*)&m_nc._local, sizeof(m_nc._local)) != 0) {
        std::cerr << "Failed to bind socket: " << strerror(errno) << std::endl;
        closesocket(m_nc._local_sock);
        m_nc._local_sock = INVALID_SOCKET;
        return FX_FALSE;
    }

    // 设置目标地址
    memset(&m_nc._to, 0, sizeof(m_nc._to));
    char ip_str[100];
    sprintf(ip_str, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
    m_nc._toLen = sizeof(struct sockaddr_in);
    m_nc._to.sin_family = AF_INET;
    m_nc._to.sin_port = htons(port);
    
    // 使用 inet_pton 替代 inet_addr（更安全）
    if (inet_pton(AF_INET, ip_str, &m_nc._to.sin_addr) <= 0) {
        std::cerr << "Invalid IP address: " << ip_str << std::endl;
        closesocket(m_nc._local_sock);
        m_nc._local_sock = INVALID_SOCKET;
        return FX_FALSE;
    }

    // 在 Linux 中，我们使用同一个 socket 进行发送和接收
    // 不需要创建单独的发送 socket
    m_nc._tosock_ = m_nc._local_sock;

    m_LinkTag = FX_TRUE;
    return FX_TRUE;
}

FX_BOOL CMarvNetAgent::OnRecv() {
    if (m_LinkTag == FX_FALSE) {
        return FX_FALSE;
    }
    
    m_buf.m_Rlen = 0;
    socklen_t addr_len = sizeof(m_nc._local);
    
    // 使用非阻塞接收
    fd_set readfds;
    struct timeval timeout;
    
    FD_ZERO(&readfds);
    FD_SET(m_nc._local_sock, &readfds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms 超时
    
    int ready = select(m_nc._local_sock + 1, &readfds, NULL, NULL, &timeout);
    if (ready <= 0) {
        return FX_FALSE;  // 超时或错误
    }
    
    m_buf.m_Rlen = recvfrom(m_nc._local_sock, m_buf.m_Recvbuf, sizeof(m_buf.m_Recvbuf), 
                           0, (struct sockaddr*)&m_nc._local, &addr_len);
    
    if (m_buf.m_Rlen <= 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Recv failed: " << strerror(errno) << std::endl;
        }
        m_buf.m_Rlen = 0;
        return FX_FALSE;
    }
    
    // 检查是否是期望的发送方
    if (m_nc._local.sin_addr.s_addr == m_nc._to.sin_addr.s_addr &&
        m_nc._local.sin_port == m_nc._to.sin_port) {
        m_nc._from_valid = 1;
    } else {
        m_nc._from_valid = 0;
    }
    
    return FX_TRUE;
}


FX_BOOL  CMarvNetAgent::OnSend()
{
	if (m_LinkTag == FX_FALSE)
	{
        return FX_FALSE;
    }
	if (m_buf.m_STag != 100)
	{
        return FX_FALSE;
    }
	if (m_buf.m_Slen <= 0)
	{
        m_buf.m_Slen = 0;
        m_buf.m_STag = 0;
        return FX_FALSE;
    }
    
    FX_INT32L send_len = 0;
    if (m_buf.m_Slen >= 0) {
        send_len = sendto(m_nc._tosock_, m_buf.m_SendBuf, m_buf.m_Slen, 0,
                         (struct sockaddr*)&m_nc._to, sizeof(m_nc._to));
    }
    
    m_buf.m_STag = 0;
    if (send_len != m_buf.m_Slen) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            // std::cerr << "Send failed: " << strerror(errno) << std::endl;
        }
        // m_buf.m_Slen = 0;
        return FX_FALSE;
    }
    m_buf.m_Slen = 0;
    return FX_TRUE;
}


FX_BOOL  CMarvNetAgent::OnSendLink()
{
	if (m_LinkTag == FX_FALSE)
	{
        return FX_FALSE;
    }
    char buf[2];
    buf[0] = 'F';
    buf[1] = 'X';
    
    FX_INT32L send_len = sendto(m_nc._tosock_, buf, 2, 0,
                               (struct sockaddr*)&m_nc._to, sizeof(m_nc._to));
    
    if (send_len != 2) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << "Send link failed: " << strerror(errno) << std::endl;
        }
        return FX_FALSE;
    }
    
    return FX_TRUE;
}

//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CACB::CACB()
{
	init_tag_ = false;
	base_ = NULL;
	item_num = 0;

	size_ = 10240;
	base_ = (unsigned char*)malloc(size_);

	init_tag_ = true;
	write_pos_ = 1;
	read_pos_ = 0;

	write_lock_ = 0;
	read_lock_ = 0;
	buf_serial_ = 0;

	item_num = 0;



}

CACB::~CACB()
{
	if (init_tag_ == true)
	{
		free(base_);
	}
}
long CACB::OnGetStoreNum()
{
	return item_num;
}

bool CACB::WriteBuf(unsigned char* data_ptr, long size_int)
{
	if (size_int < 1 || init_tag_ == false)
	{
		return false;
	}
	if (write_lock_ != 0)
	{
		return false;
	}
	write_lock_ = 1;

	long emptysize;
	long wpos = write_pos_;
	long rpos = read_pos_;

	unsigned long tmpserial = buf_serial_;
	tmpserial++;
	if (tmpserial >= 100000000)
	{
		tmpserial = 0;
	}


	if (wpos < rpos)
	{
		emptysize = rpos - wpos - 1;
		if (emptysize < size_int + 6)
		{
			write_lock_ = 0;
			return false;
		}
		base_[wpos] = size_int / 256;
		base_[wpos + 1] = size_int % 256;

		base_[wpos + 2] = (unsigned char)(tmpserial / 0x1000000);
		base_[wpos + 3] = (unsigned char)((tmpserial % 0x1000000) / 0x10000);
		base_[wpos + 4] = (unsigned char)((tmpserial % 0x10000) / 0x100);
		base_[wpos + 5] = (unsigned char)((tmpserial % 0x100));


		memcpy(&base_[wpos + 6], data_ptr, size_int);
		wpos += 6;
		wpos += size_int;
		write_pos_ = wpos;

		buf_serial_ = tmpserial;
		write_lock_ = 0;
		item_num++;
		return true;
	}
	else
	{
		long epos = size_ - wpos;
		emptysize = epos + rpos - 1;

		if (emptysize < size_int + 6)
		{
			write_lock_ = 0;
			return false;
		}

		base_[wpos] = size_int / 256;
		wpos++;
		wpos %= size_;
		base_[wpos] = size_int % 256;
		wpos++;
		wpos %= size_;

		base_[wpos] = (unsigned char)(tmpserial / 0x1000000);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)((tmpserial % 0x1000000) / 0x10000);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)((tmpserial % 0x10000) / 0x100);
		wpos++;
		wpos %= size_;
		base_[wpos] = (unsigned char)((tmpserial % 0x100));
		wpos++;
		wpos %= size_;


		epos -= 6;

		if (epos <= size_int)
		{
			if (epos > 0)
			{
				memcpy(&base_[wpos], data_ptr, epos);
				if (size_int - epos > 0)
				{
					memcpy(&base_[0], &data_ptr[epos], size_int - epos);
				}
			}
			else
			{
				memcpy(&base_[wpos], data_ptr, size_int);
			}
		}
		else
		{
			memcpy(&base_[wpos], data_ptr, size_int);
		}
		wpos += size_int;
		wpos %= size_;
		write_pos_ = wpos;

		buf_serial_ = tmpserial;
		write_lock_ = 0;
		item_num++;
		return true;
	}
}

long CACB::ReadBuf(unsigned char* data_ptr, long size_int)
{
	if (init_tag_ == false)
	{
		return -1;
	}
	if (read_lock_ != 0)
	{
		return -1;
	}
	read_lock_ = 1;

	long wpos = write_pos_;
	long rpos = read_pos_;
	rpos++;
	rpos %= size_;
	if (rpos == wpos)
	{
		read_lock_ = 0;
		return 0;
	}

	long sizetmp;
	sizetmp = base_[rpos] * 256;
	rpos++;
	rpos %= size_;
	sizetmp += base_[rpos];
	if (size_int < sizetmp)
	{
		read_lock_ = 0;
		return -2;
	}

	rpos++;
	rpos %= size_;


	rpos += 4;
	rpos %= size_;

	long explen = size_ - rpos;
	if (explen <= sizetmp)
	{
		memcpy(data_ptr, &base_[rpos], explen);
		if (sizetmp - explen > 0)
		{
			memcpy(&data_ptr[explen], base_, sizetmp - explen);
		}
	}
	else
	{
		memcpy(data_ptr, &base_[rpos], sizetmp);
	}
	rpos += (sizetmp - 1);
	rpos %= size_;
	read_pos_ = rpos;
	read_lock_ = 0;

	item_num--;
	return sizetmp;
}


bool CACB::Empty()
{
	if (init_tag_ == false)
	{
		return false;
	}
	if (read_lock_ != 0 || write_lock_ != 0)
	{
		return false;
	}

	read_lock_ = 1;
	write_lock_ = 1;

	read_pos_ = 0;
	write_pos_ = 1;

	write_lock_ = 0;
	read_lock_ = 0;


	item_num = 0;

	return true;
}






//////////////////////////////////////////////////////////////////////////////////////////////






CRobot::CRobot()
{
    m_LinkTag = FX_FALSE;
    m_RobotRTUpdateTag = 0;
    m_RobotSGUpdateTag = 0;
    m_RobotRTRecvTag = 0;
    m_RobotSGRecvTag = 0;
    m_ParaSerial = 0;
    // m_TimeEventID = 0;
    
	memset(m_SendBuf1, 0, 600);
	memset(m_SendBuf2, 0, 600);

	pDDSS1 = (DDSS * )m_SendBuf1;
	pDDSS2 = (DDSS*)m_SendBuf2;
    
    // 初始化网络（Linux 下无操作）
    socket_init();
}

CRobot::~CRobot() {
    if(m_InsRobot->m_LinkTag == FX_TRUE){
		timer_delete(m_InsRobot->robot_timer);
	}
	usleep(10000);
}

// /////////////////////

bool CRobot::OnClearChDataA()
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB1.ReadBuf((unsigned char*)&t, si);
	while (num > 0)
	{
		num = m_InsRobot->m_ACB1.ReadBuf((unsigned char*)&t, si);
	}

	return true;
}

bool CRobot::OnClearChDataB()
{
	if (m_InsRobot == NULL)
	{
		return false;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB2.ReadBuf((unsigned char*)&t, si);
	while (num > 0)
	{
		num = m_InsRobot->m_ACB2.ReadBuf((unsigned char*)&t, si);
	}

	return true;
}



long CRobot::OnGetChDataA(unsigned char data_ptr[256], long* ret_ch)
{
	if (m_InsRobot == NULL)
	{
		return 0;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB1.ReadBuf((unsigned char*)&t, si);
	if (num == 0)
	{
		return num;
	}
	memset(data_ptr, 0, 256);
	*ret_ch = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);
	return t.m_Size;

}

long CRobot::OnGetChDataB(unsigned char data_ptr[512], long* ret_ch)
{
	if (m_InsRobot == NULL)
	{
		return 0;
	}
	DDSS t;
	long si = sizeof(DDSS);
	long num = m_InsRobot->m_ACB2.ReadBuf((unsigned char*)&t, si);
	if (num == 0)
	{
		return num;
	}
	memset(data_ptr, 0, 256);
	*ret_ch = t.m_SUB_CH;
	memcpy(data_ptr, t.m_Data, t.m_Size);
	return t.m_Size;
}




bool CRobot::OnSetChDataA(unsigned char* data_ptr, long size_int, long set_ch)
{
	if (m_InsRobot == NULL)
	{
		return 0;
	}
	if (size_int <= 0 || size_int > 256)
	{
		return false;
	}
	if (m_InsRobot == NULL)
	{
		return false;
	}

	long serial = m_InsRobot->pDDSS1->m_Serial + 1;
	if (serial > 1000000)
	{
		serial = 1;
	}
	m_InsRobot->pDDSS1->m_Serial = serial;
	m_InsRobot->pDDSS1->m_Size = size_int;
	m_InsRobot->pDDSS1->m_SUB_CH = set_ch;
	memcpy(m_InsRobot->pDDSS1->m_Data, data_ptr, size_int);
	memcpy(m_InsRobot->m_Flange_NA0.m_buf.m_SendBuf, (unsigned char*)m_InsRobot->pDDSS1, sizeof(DDSS));
	m_InsRobot->m_Flange_NA0.m_buf.m_Slen = sizeof(DDSS);
	m_InsRobot->m_Flange_NA0.m_buf.m_STag = 100;
	return (FX_TRUE == m_InsRobot->m_Flange_NA0.OnSend());

	return true;
}



bool CRobot::OnSetChDataB(unsigned char* data_ptr, long size_int, long set_ch)
{
	if (m_InsRobot == NULL)
	{
		return 0;
	}
	if (size_int <= 0 || size_int > 256)
	{
		return false;
	}
	if (m_InsRobot == NULL)
	{
		return false;
	}

	long serial = m_InsRobot->pDDSS1->m_Serial + 1;
	if (serial > 1000000)
	{
		serial = 1;
	}
	m_InsRobot->pDDSS2->m_Serial = serial;
	m_InsRobot->pDDSS2->m_Size = size_int;
	m_InsRobot->pDDSS2->m_SUB_CH = set_ch;
	memcpy(m_InsRobot->pDDSS2->m_Data, data_ptr, size_int);
	memcpy(m_InsRobot->m_Flange_NA1.m_buf.m_SendBuf, (unsigned char*)m_InsRobot->pDDSS2, sizeof(DDSS));
	m_InsRobot->m_Flange_NA1.m_buf.m_Slen = sizeof(DDSS);
	m_InsRobot->m_Flange_NA1.m_buf.m_STag = 100;
	return (FX_TRUE== m_InsRobot->m_Flange_NA1.OnSend());
}

// ////////////////////

CRobot* CRobot::GetIns()
{
	if (m_InsRobot == NULL)
	{
        m_InsRobot = new CRobot();
    }
    return m_InsRobot;
}

long tmpct = 0;
void  CallBackFunc(union sigval v)
{
	m_InsRobot->DoRecv();
	m_InsRobot->DoSend();
	tmpct ++;
	if(tmpct % 1000 == 0)
	{
		//if(m_InsRobot->m_LocalLogTag == true) printf("tmpct  = %d\n",tmpct);
	}
}

bool CRobot::OnTest() {
    GetIns();
    if (m_InsRobot->m_LinkTag == FX_FALSE) {
        return false;
    }

    m_InsRobot->m_RT_NA.OnSendLink();
    m_InsRobot->m_SG_NA.OnSendLink();
    
    return true;
}

bool CRobot::OnLinkTo(FX_UCHAR ip1, FX_UCHAR ip2, FX_UCHAR ip3, FX_UCHAR ip4)
{
	GetIns();
	if (m_InsRobot->m_LinkTag == FX_TRUE)
	{
		return false;
	}
	if (m_InsRobot->m_RT_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3721) == FX_FALSE)
	{
		return false;
	}
	if (m_InsRobot->m_SG_NA.OnLinkTo(ip1, ip2, ip3, ip4, 3722) == FX_FALSE)
	{
		return false;
	}

	if (m_InsRobot->m_Flange_NA0.OnLinkTo(ip1, ip2, ip3, ip4, 3723) == FX_FALSE)
	{
		return false;
	}
	if (m_InsRobot->m_Flange_NA1.OnLinkTo(ip1, ip2, ip3, ip4, 3724) == FX_FALSE)
	{
		return false;
	}

	m_InsRobot->m_LinkTag = FX_TRUE;

	{
		struct sigevent evp;
		struct itimerspec ts;
		int ret;
		memset   (&evp, 0, sizeof(evp));
		evp.sigev_value.sival_ptr = &m_InsRobot->robot_timer;
		evp.sigev_notify = SIGEV_THREAD;
		evp.sigev_notify_function = CallBackFunc;
		evp.sigev_value.sival_int = 0;  
		ret = timer_create(CLOCK_REALTIME, &evp, &m_InsRobot->robot_timer);
		if( ret){
			return false;
		}
	
		ts.it_interval.tv_sec = 0;
		ts.it_interval.tv_nsec = 1000000;
		ts.it_value.tv_sec = 0;
		ts.it_value.tv_nsec = 1000000;
		ret = timer_settime(m_InsRobot->robot_timer, TIMER_ABSTIME, &ts, NULL);
		if( ret )
		{
			return false;
		}
	}

	return true;

}

long CRobot::OnStartLink() {
    GetIns();
    if (m_InsRobot->m_LinkTag == FX_FALSE) {
        printf("Link tag is false\n");
        return -1;
    }
    
    m_InsRobot->m_RobotRTRecvTag = 0;
    m_InsRobot->m_RobotSGRecvTag = 0;
    
    m_InsRobot->m_RT_NA.OnSendLink();
    m_InsRobot->m_SG_NA.OnSendLink();
    
    long cnt = 0;
    
    // Linux 下使用 usleep（微秒）
    usleep(2000);  // 2ms
    long ret = 2;
    
    while (cnt < 1000) {
        cnt++;
        if (m_InsRobot->m_RobotRTRecvTag != 0 && m_InsRobot->m_RobotSGRecvTag != 0) {
            return ret;
        }
        usleep(1000);  // 1ms
        ret++;
    }
    printf("Link timeout\n");
    return -1;
}

void CRobot::DoRecv() {
    GetIns();
    if (m_InsRobot->m_LinkTag == FX_FALSE) {
        return;
    }
    
    // 接收 RT 数据
    if (m_InsRobot->m_RT_NA.OnRecv()) {
        if (m_InsRobot->m_RT_NA.m_buf.m_Rlen == sizeof(ROBOT_RT) + 2) {
            m_InsRobot->m_RobotRTRecvTag = 1;
			memcpy(&m_InsRobot->m_RobotRT, &m_InsRobot->m_RT_NA.m_buf.m_Recvbuf[2], m_InsRobot->m_RT_NA.m_buf.m_Rlen);
            m_InsRobot->m_RT_NA.m_buf.m_Rlen = 0;
            m_InsRobot->m_RobotRTUpdateTag++;
			if (m_InsRobot->m_RobotRTUpdateTag > 100)
			{
                m_InsRobot->m_RobotRTUpdateTag = 100;
            }
        }
    }
	m_InsRobot->m_SG_NA.OnRecv();
	if (m_InsRobot->m_SG_NA.m_buf.m_Rlen > 0)
	{
		if (m_InsRobot->m_SG_NA.m_buf.m_Rlen == sizeof(ROBOT_SG)+2)
		{
            m_InsRobot->m_RobotSGRecvTag = 1;
			memcpy(&m_InsRobot->m_RobotSG, &m_InsRobot->m_SG_NA.m_buf.m_Recvbuf[2], m_InsRobot->m_SG_NA.m_buf.m_Rlen);
            m_InsRobot->m_SG_NA.m_buf.m_Rlen = 0;
            m_InsRobot->m_RobotSGUpdateTag++;
			if (m_InsRobot->m_RobotSGUpdateTag > 100)
			{
                m_InsRobot->m_RobotSGUpdateTag = 100;
            }
        }
    }

	m_InsRobot->m_Flange_NA0.OnRecv();
	while (m_InsRobot->m_Flange_NA0.m_buf.m_Rlen > 0)
	{
		if (m_InsRobot->m_Flange_NA0.m_buf.m_Rlen == sizeof(DDSS))
		{
			m_InsRobot->m_ACB1.WriteBuf((unsigned char *) m_InsRobot->m_Flange_NA0.m_buf.m_Recvbuf, sizeof(DDSS));
		}
		m_InsRobot->m_Flange_NA0.m_buf.m_Rlen = 0;
		m_InsRobot->m_Flange_NA0.OnRecv();
	}


	m_InsRobot->m_Flange_NA1.OnRecv();
	while (m_InsRobot->m_Flange_NA1.m_buf.m_Rlen > 0)
	{
		if (m_InsRobot->m_Flange_NA1.m_buf.m_Rlen == sizeof(DDSS))
		{
			m_InsRobot->m_ACB2.WriteBuf((unsigned char*)m_InsRobot->m_Flange_NA1.m_buf.m_Recvbuf, sizeof(DDSS));
		}
		m_InsRobot->m_Flange_NA1.m_buf.m_Rlen = 0;
		m_InsRobot->m_Flange_NA1.OnRecv();
	}

}


void CRobot::DoSend()
{
    GetIns();
    m_InsRobot->m_RT_NA.OnSend();
}

bool CRobot::OnClearSet()
{
    GetIns();
	if (m_InsRobot->m_RT_NA.m_buf.m_STag == 100)
	{///////////////
        return false;
    }
    m_InsRobot->m_RT_NA.m_buf.m_STag = 0;
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[0] = 'F';
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[1] = 'a';
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[2] = 0;
    m_InsRobot->m_RT_NA.m_buf.m_Slen = 3;
    
    return true;
}

bool CRobot::OnSetSend()
{
	if (m_InsRobot->m_RT_NA.m_buf.m_STag == 100)
	{
        return false;
    }
	m_InsRobot->m_RT_NA.m_buf.m_STag = 100;
    
    return true;
}

bool CRobot::OnSetState(long ins, long cmd_state)
{
    GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
        return false;
    }
    
    long add_size = 2 + sizeof(FX_INT16);
    
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1400)
	{
        return false;
    }
    
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (FX_UCHAR)ins;
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = sizeof(FX_INT16);
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    
    FX_INT16* pv = (FX_INT16*)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
    *pv = (FX_INT16)cmd_state;
    
    m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(FX_INT16);
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    
    FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[2];
    (*pnum)++;
    
    return true;
}

bool CRobot::OnSetFLoat(long ins, long num, double* pdata)
{
    GetIns();
	if (m_InsRobot->m_LinkTag == FX_FALSE)
	{
        return false;
    }
    
    long add_size = 2 + sizeof(float) * num;
    
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1400)
	{
        return false;
    }
    
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (FX_UCHAR)ins;
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = sizeof(float) * num;
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    
    float* pv = (float*)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
    for (long i = 0; i < num; i++) {
        pv[i] = (float)pdata[i];
    }
    
    m_InsRobot->m_RT_NA.m_buf.m_Slen += sizeof(float) * num;
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    
    FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[2];
    (*pnum)++;
    
    return true;
}

bool CRobot::OnSetRawData(long ins, long num, unsigned char* pdata) {
    GetIns();
    if (m_InsRobot->m_LinkTag == FX_FALSE) {
        return false;
    }
    
    long add_size = 2 + num;
    
	if (add_size + m_InsRobot->m_RT_NA.m_buf.m_Slen >= 1400)
	{
        return false;
    }
    
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = (FX_UCHAR)ins;
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = num;
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    
    unsigned char* pv = (unsigned char*)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen];
    for (long i = 0; i < num; i++) {
        pv[i] = pdata[i];
    }
    
    m_InsRobot->m_RT_NA.m_buf.m_Slen += num;
    m_InsRobot->m_RT_NA.m_buf.m_SendBuf[m_InsRobot->m_RT_NA.m_buf.m_Slen] = 'X';
    m_InsRobot->m_RT_NA.m_buf.m_Slen++;
    
    FX_UCHAR* pnum = (FX_UCHAR*)&m_InsRobot->m_RT_NA.m_buf.m_SendBuf[2];
    (*pnum)++;
    
    return true;
}

bool CRobot::OnSetArmState0(long cmd_state)
{
    return CRobot::OnSetState(UDP_ARM0_RT_CmdState, cmd_state);
}

bool CRobot::OnSetArmState1(long cmd_state)
{
    return CRobot::OnSetState(UDP_ARM1_RT_CmdState, cmd_state);
}

bool CRobot::OnSetHeadState(long cmd_state)
{
    return CRobot::OnSetState(UDP_HEAD_RT_CmdState, cmd_state);
}
bool CRobot::OnSetBodyState(long cmd_state)
{
    return CRobot::OnSetState(UDP_BODY_RT_CmdState, cmd_state);
}

bool CRobot::OnClearArmErr0()
{
    CRobot* ins = CRobot::GetIns();

    ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
        ins->m_ParaSerial = 1;
    }
    long serial = ins->m_ParaSerial;
    OP_SET buf;
	memset(&buf,0,sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM0_RESET;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);

}
bool CRobot::OnClearArmErr1()
{
	CRobot* ins = CRobot::GetIns();

    ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
        ins->m_ParaSerial = 1;
    }
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM1_RESET;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnClearHeadErr()
{
    CRobot* ins = CRobot::GetIns();

    ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
        ins->m_ParaSerial = 1;
    }
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_HEAD_RESET;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}
bool CRobot::OnClearBodyErr()
{
	CRobot* ins = CRobot::GetIns();

    ins->m_ParaSerial++;
	if (ins->m_ParaSerial >= 99)
	{
        ins->m_ParaSerial = 1;
    }
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_BODY_RESET;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEMGArm0() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM0_EMCY;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEMGArm1() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM1_EMCY;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEMGHead() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_HEAD_EMCY;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEMGBody() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_BODY_EMCY;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEncResetArm0() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM0_ENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEncResetArm1() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM1_ENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEncResetHead() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_HEAD_ENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnEncResetBody() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_BODY_ENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnExEncResetArm0() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM0_EXTENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnExEncResetArm1() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_ARM1_EXTENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnExEncResetHead() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_HEAD_EXTENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnExEncResetBody() {
    CRobot* ins = CRobot::GetIns();
    ins->m_ParaSerial++;
    if (ins->m_ParaSerial >= 99) {
        ins->m_ParaSerial = 1;
    }
    
    long serial = ins->m_ParaSerial;
    OP_SET buf;
    memset(&buf, 0, sizeof(OP_SET));
    buf.m_OpCmdSerial = serial;
    buf.m_OpIns = OPINS_BODY_EXTENC_RESET_OFFSET;
    buf.m_OpValueI = 0xFF;
    
    return OnSetRawData(UDP_OPERATION, 44, (unsigned char*)&buf);
}

bool CRobot::OnSetArmCmdPos0(double cmdpos[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = cmdpos[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_RT_CmdJointPos, 7, v);
}

bool CRobot::OnSetArmCmdPos1(double cmdpos[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = cmdpos[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_RT_CmdJointPos, 7, v);
}

bool CRobot::OnSetHeadCmdPos(double cmdpos[3]) {
    double v[7] = { 0 };
    for (int i = 0; i < 3; i++) {
        v[i] = cmdpos[i];
    }
    return CRobot::OnSetFLoat(UDP_HEAD_RT_CmdPos, 3, v);
}

bool CRobot::OnSetBodyCmdPos(double cmdpos[6]) {
    double v[7] = { 0 };
    for (int i = 0; i < 6; i++) {
        v[i] = cmdpos[i];
    }
    return CRobot::OnSetFLoat(UDP_BODY_RT_CmdPos, 6, v);
}

bool CRobot::OnSetArmVelRatio0(double cmd_vel_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_vel_lmt;
    if (v[0] < 0.01) {
        v[0] = 0.01;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_VelRatio, 1, v);
}

bool CRobot::OnSetArmVelRatio1(double cmd_vel_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_vel_lmt;
    if (v[0] < 0.01) {
        v[0] = 0.01;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_VelRatio, 1, v);
}

bool CRobot::OnSetArmAccRatio0(double cmd_acc_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_acc_lmt;
    if (v[0] < 1) {
        v[0] = 1;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_AccRatio, 1, v);
}

bool CRobot::OnSetArmAccRatio1(double cmd_acc_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_acc_lmt;
    if (v[0] < 1) {
        v[0] = 1;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_AccRatio, 1, v);
}

bool CRobot::OnSetArmJointK0(double jk[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = jk[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_JointK, 7, v);
}

bool CRobot::OnSetArmJointK1(double jk[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = jk[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_JointK, 7, v);
}

bool CRobot::OnSetArmJointD0(double jd[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = jd[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_JointD, 7, v);
}

bool CRobot::OnSetArmJointD1(double jd[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = jd[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_JointD, 7, v);
}

bool CRobot::OnSetArmToolK0(double tk[6]) {
    double v[6] = { 0 };
    for (int i = 0; i < 6; i++) {
        v[i] = tk[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_TOOLK, 6, v);
}

bool CRobot::OnSetArmToolK1(double tk[6]) {
    double v[6] = { 0 };
    for (int i = 0; i < 6; i++) {
        v[i] = tk[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_TOOLK, 6, v);
}

bool CRobot::OnSetArmToolD0(double td[10]) {
    double v[10] = { 0 };
    for (int i = 0; i < 10; i++) {
        v[i] = td[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_TOOLD, 10, v);
}

bool CRobot::OnSetArmToolD1(double td[10]) {
    double v[10] = { 0 };
    for (int i = 0; i < 10; i++) {
        v[i] = td[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_TOOLD, 10, v);
}

bool CRobot::OnSetHeadVelRatio(double cmd_vel_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_vel_lmt;
    if (v[0] < 1) {
        v[0] = 1;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_HEAD_SG_VelRatio, 1, v);
}

bool CRobot::OnSetHeadAccRatio(double cmd_acc_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_acc_lmt;
    if (v[0] < 1) {
        v[0] = 1;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_HEAD_SG_AccRatio, 1, v);
}

bool CRobot::OnSetBodyVelRatio(double cmd_vel_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_vel_lmt;
    if (v[0] < 1) {
        v[0] = 1;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_BODY_SG_VelRatio, 1, v);
}

bool CRobot::OnSetBodyAccRatio(double cmd_acc_lmt) {
    double v[7] = { 0 };
    v[0] = cmd_acc_lmt;
    if (v[0] < 1) {
        v[0] = 1;
    }
    if (v[0] > 100.0) {
        v[0] = 100.0;
    }
    return CRobot::OnSetFLoat(UDP_BODY_SG_AccRatio, 1, v);
}

bool CRobot::OnSetArmCartK0(double ck[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = ck[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_CartK, 7, v);
}

bool CRobot::OnSetArmCartK1(double ck[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = ck[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_CartK, 7, v);
}

bool CRobot::OnSetArmCartD0(double cd[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = cd[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM0_SG_CartD, 7, v);
}

bool CRobot::OnSetArmCartD1(double cd[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = cd[i];
    }
    return CRobot::OnSetFLoat(UDP_ARM1_SG_CartD, 7, v);
}

bool CRobot::OnSetHandState0(long cmd_state) {
    return CRobot::OnSetState(UDP_HAND0_RT_CmdState, cmd_state);
}

bool CRobot::OnSetHandState1(long cmd_state) {
    return CRobot::OnSetState(UDP_HAND1_RT_CmdState, cmd_state);
}

bool CRobot::OnSetAGVState(long cmd_state) {
    return CRobot::OnSetState(UDP_AGV_RT_CmdState, cmd_state);
}

bool CRobot::OnSetArmJointCmdToq0(double cmdtoq[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = cmdtoq[i];
    }
    return CRobot::OnSetFLoat(30, 7, v);
}

bool CRobot::OnSetArmJointCmdToq1(double cmdtoq[7]) {
    double v[7] = { 0 };
    for (int i = 0; i < 7; i++) {
        v[i] = cmdtoq[i];
    }
    return CRobot::OnSetFLoat(31, 7, v);
}

bool CRobot::OnSetArmForceCtrlCmd0(double force_ctrl_cmd[5]) {
    double v[7] = { 0 };
    for (int i = 0; i < 5; i++) {
        v[i] = force_ctrl_cmd[i];
    }
    return CRobot::OnSetFLoat(32, 5, v);
}

bool CRobot::OnSetArmForceCtrlCmd1(double force_ctrl_cmd[5]) {
    double v[7] = { 0 };
    for (int i = 0; i < 5; i++) {
        v[i] = force_ctrl_cmd[i];
    }
    return CRobot::OnSetFLoat(33, 5, v);
}

bool CRobot::OnSetArmTorqueCtrlCmd0(double toq_ctrl_cmd[5]) {
    double v[7] = { 0 };
    for (int i = 0; i < 5; i++) {
        v[i] = toq_ctrl_cmd[i];
    }
    return CRobot::OnSetFLoat(34, 5, v);
}

bool CRobot::OnSetArmTorqueCtrlCmd1(double toq_ctrl_cmd[5]) {
    double v[7] = { 0 };
    for (int i = 0; i < 5; i++) {
        v[i] = toq_ctrl_cmd[i];
    }
    return CRobot::OnSetFLoat(35, 5, v);
}

bool CRobot::OnSetAGVVel(double vel[4]) {
    double v[7] = { 0 };
    for (int i = 0; i < 4; i++) {
        v[i] = vel[i];
    }
    return CRobot::OnSetFLoat(38, 4, v);
}

bool CRobot::OnSetAGVCmdPos(double cmdpos[4]) {
    double v[7] = { 0 };
    for (int i = 0; i < 4; i++) {
        v[i] = cmdpos[i];
    }
    return CRobot::OnSetFLoat(39, 4, v);
}

bool CRobot::OnSetBodyPDP(double p[6]) {
    double v[7] = { 0 };
    for (int i = 0; i < 6; i++) {
        v[i] = p[i];
    }
    return CRobot::OnSetFLoat(UDP_BODY_SG_PDK, 6, v);
}

bool CRobot::OnSetBodyPDD(double d[6]) {
    double v[7] = { 0 };
    for (int i = 0; i < 6; i++) {
        v[i] = d[i];
    }
    return CRobot::OnSetFLoat(UDP_BODY_SG_PDD, 6, v);
}