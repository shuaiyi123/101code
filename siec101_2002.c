#include "siec101_2002.h"

//==================================================
// 名称: siec101_2002.c
// 功能: 处理S101规约的流程控制
// 编写：盖希波 2003.12.10
// 改写：2019.6.20 
//===================================================

uint8 UserFlag[10]={0}; //用户标志位
TIEC101Cfg_Handle m_pCfg;  //报文配置文件

/**
 * @description: RTU初始化，主要对RTU的一些基础配置
 * @param {none} 
 * @return: 初始化状态，返回TRUE说明初始化成功
 */
void RTU_Init()
{
    m_pCfg.bBalance=0; //非平衡传输
    m_pCfg.wProtocolID=0x0088;  //公共地址地址
    m_pCfg.bLinkAddr=0x0088;    //链路地址
    m_pCfg.bLinkAddrLen=1;       //链路地址长度：1字节
    m_pCfg.bCauseTransLen=1;     //传输原因：1字节
    m_pCfg.bInfoAddrLen=2;       //信息体地址：2字节
    m_pCfg.bAsduAddrLen=1;       //公共地址：1字节        
    m_pCfg.bTestMode=0;       //0正常模式，1测试模式  
}

/**
 * @description: 保存报文控制字
 * @param {user_funCode:保存控制字变量，funCode:接收到的控制字} 
 * @return: none
 */
void Save_CtrlCode(uint8 *user_funCode,uint8 funCode)
{
    *(user_funCode+1)=*user_funCode;  //保存上一次的控制字
    *user_funCode=funCode;   //本次接收的控制字
}
