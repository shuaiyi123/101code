#ifndef _SIEC101_2002_H
#define _SIEC101_2002_H

#include <stdio.h>
#include <stdlib.h>


#define YC_LENTH 64
#define YX_LENTH 64
//帧控制域定义
#define BIT_DIR    0x80    //传输方向位
#define BIT_PRM    0x40    //启动报文位
#define BIT_FCB    0x20    //帧计数位
#define BIT_FCV    0x10    //帧计数有效位
#define BIT_FUNC   0x0F    //功能码所占位

#define BIT_FCBFCV	0x30   //帧计数位和帧计数有效位

//功能码定义
#define  RESREMLIN 0x00 //复位远方链路
#define  RESRTU    0x01 //复位远动终端
#define  SENDAT    0x03 //传送数据
#define  REQANS    0x08 //召唤要求访问位
#define  REQLINSTA 0x09 //请求远方链路状态
#define  CALFIRDAT 0x0A //召唤一级用户数据
#define  CALSECDAT 0x0B //召唤二级用户数据

#define SF_RXDCONTROL		(UserFlag + 0) //接收本次的控制字
#define SF_LAST_FCBFCV		(UserFlag + 1) //保存上一次的控制字
#define SF_NEXT_FCBFCV      (UserFlag + 2) //期待下次的FCBFCV位

#define SF_RtuInitok        (UserFlag + 3) //RTU初始化结束标志位
#define SF_Class1YX_Flag    (UserFlag + 4) //总召一级遥信标志位 
#define SF_Class1YC_Flag    (UserFlag + 5) //总召一级遥测标志位
#define SF_RespCall_Flag    (UserFlag + 6) //总召唤命令确认标志
#define SF_AllCallEnd_Flag  (UserFlag + 7) //总召命令结束标志
//数据类型声明
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

//状态量定义
typedef enum
{
    FALSE = 0,
    TRUE = 1
} FuncStatus;

#ifndef PACKED
#define PACKED __attribute__((packed)) //不对齐，结构体的长度，就是各个成员变量长度的和
#endif

//I101S规约的固定帧长结构
typedef struct
{
    uint8 bStart;  //启动字符，一个字符
    uint8 bCtrl;   //控制域，一个字符
    uint8 bAddr;   //链路地址域（子站地址）
    uint8 bChkSum; //帧校验和
    uint8 bStop;   //结束字符
} PACKED TRxdFixFm, TTxdFixFm;

//I101S规约的可变帧长结构
typedef struct
{
    uint8 var_Start;     //启动字符
    uint8 var_Length;    //长度
    uint8 var_Length1;   //长度
    uint8 var_Start1;    //启动字符
    uint8 var_Ctrl;      //控制域
    uint8 var_Addr;      //链路地址域
    uint8 var_Type;      //类型标识
    uint8 var_Defini;    //结构限定字
    uint8 var_Reason;    //传送原因
    uint8 var_PulibAddr; //公共地址
    uint8 var_Data;      //数据开始
} PACKED TRxdVarFm, TTxdVarFm;

//接收报文缓存区
typedef struct {
    uint8 *buf;
    uint8 ReadPtr;
}RxdFrame_buf;
//发送报文缓存区
typedef struct {
    uint8 *buf;
    uint8 WritePtr;
}TxdFrame_buf;

//帧句柄，用于初始化帧结构
typedef struct   
{
    uint16 wProtocolID;   //设备地址/公共地址
    uint16 bLinkAddr;     //链路地址
    uint8 bBalance;       //非平衡传输
    uint8 bLinkAddrLen;   //链路地址长度
    uint8 bAsduAddrLen;      //公用地址长度
    uint8 bInfoAddrLen;      //信息体地址长度
    uint8 bCauseTransLen;    //传送原因长度
    uint8 bTestMode;      //测试模式,bit7:1:测试模式，0：正常模式
} PACKED TIEC101Cfg_Handle;

//共用体型，各变量共用内存
typedef union {
    TRxdFixFm fixFm;
    TRxdVarFm varFm;
} PACKED TSIEC101_2002RxdFm; //101规约接收帧结构（头部）

//共用体型，各变量共用内存
typedef union {
    TTxdFixFm fixFm;
    TTxdVarFm varFm;
} PACKED TSIEC101_2002TxdFm; //规约发送帧结构（头部）


//接收模块函数声明
FuncStatus RxdMonitor();
FuncStatus SearchFrame();
uint16 MakeAddr(uint8 low_linAddr,uint8 high_linAddr);
uint8 ChkSum(uint8* p_Addr,uint8 chek_len);
FuncStatus SwitchToAddress(uint16 LinkAddr);
FuncStatus CheckAndRetrans(uint8 bControl);
FuncStatus RxdFixFrame(); //固定帧处理
FuncStatus RxdVarFrame(); //可变帧处理
FuncStatus RxdResetLink();//0x00 复位远方链路
FuncStatus RxdReqLinkStatus();//0x09 请求远方链路状态
FuncStatus RxdClass1Data();//召唤一级用户数据
FuncStatus RxdClass2Data();//远方链路状态完好或召唤二级用户数据
void RxdCallAll();//总召唤/召唤某一组数据
FuncStatus ChkVarControl(uint8 funCode);  //检查链路服务是否完好

//发送模块函数声明
FuncStatus TxdFixFrame(uint8 bPRM, uint8 bCode);
FuncStatus Txd_Head(uint8 bPRM, uint8 bCode,uint8 bType, uint8 bReason);
FuncStatus Txd_Tail(uint8 bNum,uint16 inforAddr,uint8 *inforEle,uint8 yes_yc);
uint8 GetCtrCode(uint8 bPRM, uint8 bCode);
FuncStatus Fix_ACK();
FuncStatus Var_ACK(uint8 bcode,uint8 bType,uint8 bReason);
FuncStatus Txd_NoData();
FuncStatus Txd_NoData_E5();
FuncStatus TxdRetry();
void Txd_CallAll_Respond();   //总召激活确认
void TXd_YCData(uint8 num,uint16 inforAddr,uint8 *YCData,uint8 yes_yc);//上送遥测数据
void Txd_YXData(uint8 num,uint16 inforAddr,uint8 *YXData);//上送遥信数据
void Txd_CallAllEnd(); //总召激活结束

FuncStatus Txd_Class1();
FuncStatus Txd_Class2();
FuncStatus SearchClass1();


//控制流程模块函数声明
void RTU_Init();
void Save_CtrlCode(uint8 *user_funCode,uint8 funCode);

uint8 StartTrans(uint8 *buff,uint8 buff_Length);

#endif
