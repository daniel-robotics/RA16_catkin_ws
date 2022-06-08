/*
 *  kernel_cfg.c
 *  Wed May 04 12:04:12 2022
 *  SG Version 2.00
 *  sg.exe ./project.oil -os=ECC2 -I/home/nxtosek/toppers_osek/sg/impl_oil -template=/home/nxtosek/toppers_osek/sg/lego_nxt.sgt
 */
#include "osek_kernel.h"
#include "kernel_id.h"
#include "alarm.h"
#include "interrupt.h"
#include "resource.h"
#include "task.h"

#define __STK_UNIT VP
#define __TCOUNT_STK_UNIT(sz) (((sz) + sizeof(__STK_UNIT) - 1) / sizeof(__STK_UNIT))

#define TNUM_ALARM     3
#define TNUM_COUNTER   1
#define TNUM_ISR2      0
#define TNUM_RESOURCE  3
#define TNUM_TASK      4
#define TNUM_EXTTASK   0

const UINT8 tnum_alarm    = TNUM_ALARM;
const UINT8 tnum_counter  = TNUM_COUNTER;
const UINT8 tnum_isr2     = TNUM_ISR2;
const UINT8 tnum_resource = TNUM_RESOURCE;
const UINT8 tnum_task     = TNUM_TASK;
const UINT8 tnum_exttask  = TNUM_EXTTASK;

 /****** Object OS ******/

 /****** Object TASK ******/

const TaskType TASK_MOTORREG = 0;
const TaskType TASK_SENSORS = 1;
const TaskType TASK_LCD = 2;
const TaskType TASK_BACKGROUND = 3;

extern void TASKNAME( TASK_MOTORREG )( void );
extern void TASKNAME( TASK_SENSORS )( void );
extern void TASKNAME( TASK_LCD )( void );
extern void TASKNAME( TASK_BACKGROUND )( void );

static __STK_UNIT _stack_TASK_MOTORREG[__TCOUNT_STK_UNIT(512)];
static __STK_UNIT _stack_TASK_SENSORS[__TCOUNT_STK_UNIT(512)];
static __STK_UNIT _stack_TASK_LCD[__TCOUNT_STK_UNIT(512)];
static __STK_UNIT _stack_TASK_BACKGROUND[__TCOUNT_STK_UNIT(512)];

const Priority tinib_inipri[TNUM_TASK] = { TPRI_MINTASK + 10, TPRI_MINTASK + 3, TPRI_MINTASK + 2, TPRI_MINTASK + 1, };
const Priority tinib_exepri[TNUM_TASK] = { TPRI_MINTASK + 10, TPRI_MINTASK + 3, TPRI_MINTASK + 2, TPRI_MINTASK + 1, };
const UINT8 tinib_maxact[TNUM_TASK] = { (1) - 1, (1) - 1, (1) - 1, (1) - 1, };
const AppModeType tinib_autoact[TNUM_TASK] = { 0x00000000, 0x00000000, 0x00000000, 0x00000001, };
const FP tinib_task[TNUM_TASK] = { TASKNAME( TASK_MOTORREG ), TASKNAME( TASK_SENSORS ), TASKNAME( TASK_LCD ), TASKNAME( TASK_BACKGROUND ), };
const __STK_UNIT tinib_stk[TNUM_TASK] = { (__STK_UNIT)_stack_TASK_MOTORREG, (__STK_UNIT)_stack_TASK_SENSORS, (__STK_UNIT)_stack_TASK_LCD, (__STK_UNIT)_stack_TASK_BACKGROUND, };
const UINT16 tinib_stksz[TNUM_TASK] = { 512, 512, 512, 512, };

TaskType tcb_next[TNUM_TASK];
UINT8 tcb_tstat[TNUM_TASK];
Priority tcb_curpri[TNUM_TASK];
UINT8 tcb_actcnt[TNUM_TASK];
EventMaskType tcb_curevt[TNUM_EXTTASK+1];
EventMaskType tcb_waievt[TNUM_EXTTASK+1];
ResourceType tcb_lastres[TNUM_TASK];
DEFINE_CTXB(TNUM_TASK);

 /****** Object COUNTER ******/

const CounterType SysTimerCnt = 0;

const TickType cntinib_maxval[TNUM_COUNTER] = { 100000, };
const TickType cntinib_maxval2[TNUM_COUNTER] = { 200001, };
const TickType cntinib_tickbase[TNUM_COUNTER] = { 1, };
const TickType cntinib_mincyc[TNUM_COUNTER] = { 1, };

AlarmType cntcb_almque[TNUM_COUNTER];
TickType cntcb_curval[TNUM_COUNTER];

 /****** Object ALARM ******/

const AlarmType ALARM_MOTORREG = 0;
const AlarmType ALARM_SENSORS = 1;
const AlarmType ALARM_LCD = 2;

DeclareTask(TASK_MOTORREG);
static void _activate_alarm_ALARM_MOTORREG( void );
static void _activate_alarm_ALARM_MOTORREG( void )
{ (void)ActivateTask( TASK_MOTORREG ); }

DeclareTask(TASK_SENSORS);
static void _activate_alarm_ALARM_SENSORS( void );
static void _activate_alarm_ALARM_SENSORS( void )
{ (void)ActivateTask( TASK_SENSORS ); }

DeclareTask(TASK_LCD);
static void _activate_alarm_ALARM_LCD( void );
static void _activate_alarm_ALARM_LCD( void )
{ (void)ActivateTask( TASK_LCD ); }

const CounterType alminib_cntid[TNUM_ALARM] = { 0, 0, 0, };
const FP alminib_cback[TNUM_ALARM] = { _activate_alarm_ALARM_MOTORREG, _activate_alarm_ALARM_SENSORS, _activate_alarm_ALARM_LCD, };
const AppModeType alminib_autosta[TNUM_ALARM] = { 0x00000001, 0x00000001, 0x00000001, };
const TickType alminib_almval[TNUM_ALARM] = { 1, 1, 1, };
const TickType alminib_cycle[TNUM_ALARM] = { 20, 7, 100, };

AlarmType almcb_next[TNUM_ALARM];
AlarmType almcb_prev[TNUM_ALARM];
TickType almcb_almval[TNUM_ALARM];
TickType almcb_cycle[TNUM_ALARM];

 /****** Object RESOURCE ******/

const ResourceType RES_MOTORS = 0;
const ResourceType RES_SENSORS = 1;
const ResourceType RES_LCD = 2;

const Priority resinib_ceilpri[TNUM_RESOURCE] = { TPRI_MINTASK + 10, TPRI_MINTASK + 3, TPRI_MINTASK + 2, };

Priority rescb_prevpri[TNUM_RESOURCE];
ResourceType rescb_prevres[TNUM_RESOURCE];

 /****** Object EVENT ******/


 /****** Object ISR ******/


#define IPL_MAXISR2 0
const IPL ipl_maxisr2 = IPL_MAXISR2;


const Priority isrinib_intpri[TNUM_ISR2+1] = { 0};
ResourceType isrcb_lastres[TNUM_ISR2+1];

 /****** Object APPMODE ******/

void object_initialize( void )
{
	alarm_initialize();
	resource_initialize();
	task_initialize();
}


/*
 *  TOPPERS/OSEK Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      OSEK Kernel
 * 
 *  Copyright (C) 2006 by Witz Corporation, JAPAN
 * 
 *  上記著作権者は，以下の (1)～(4) の条件か，Free Software Foundation 
 *  によって公表されている GNU General Public License の Version 2 に記
 *  述されている条件を満たす場合に限り，本ソフトウェア（本ソフトウェア
 *  を改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，その適用可能性も
 *  含めて，いかなる保証も行わない．また，本ソフトウェアの利用により直
 *  接的または間接的に生じたいかなる損害に関しても，その責任を負わない．
 * 
 */




