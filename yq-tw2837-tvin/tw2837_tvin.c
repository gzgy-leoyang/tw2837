#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#define DEBUG    1 

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <asm/uaccess.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#include "tw2837_ioctl.h"

// #define _CAMERA_V (0)  // 竖屏摄像头
#define _CAMERA_H (1)  // 横屏摄像头

// 窗口默认参数，可以通过　ioctl　进行配置，以适配不同的摄像头
static int h_delay = 32;
static int h_active = 720;

static int v_delay = 6;
static int v_active = 288;

struct sensor {
	struct sensor_data sen;
	v4l2_std_id std_id;
	int rev_id;
} tw2837_data;

typedef enum {
	TW2837_NTSC = 0,	/*!< Locked on (M) NTSC video signal. */
	TW2837_PAL,		/*!< (B, G, H, I, N)PAL video signal. */
	TW2837_NOT_LOCKED,	/*!< Not locked on a signal. */
} video_fmt_idx;

struct video_fmt_t {
	int v4l2_id;
	char name[16];
	u16 raw_width;
	u16 raw_height;
	u16 active_width;
	u16 active_height;
	int frame_rate;
};

static const struct video_fmt_t video_fmts[] = {
	{
	 .v4l2_id = V4L2_STD_NTSC,
	 .name = "NTSC",
	 .raw_width = 720,
	 .raw_height = 525,
	 .active_width = 720,
	 .active_height = 480,
	 .frame_rate = 30,
	 },
	{
	 .v4l2_id = V4L2_STD_PAL,
	 .name = "PAL",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 25,
	 },
	{
	 .v4l2_id = V4L2_STD_ALL,
	 .name = "Autodetect",
	 .raw_width = 720,
	 .raw_height = 625,
	 .active_width = 720,
	 .active_height = 576,
	 .frame_rate = 0,
	 },
};

static video_fmt_idx video_idx = TW2837_PAL ;

static DEFINE_MUTEX(mutex);

#define IF_NAME					"tw2837"

#define HW_DEINT /* Enable hardware deinterlacer */

static const struct v4l2_queryctrl tw2837_qctrl[] = {
	{
	.id = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,
	.maximum = 255,
	.step = 1,
	.default_value = 127,
	.flags = 0,
	}, {
	.id = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,
	.maximum = 255,
	.step = 0x1,
	.default_value = 127,
	.flags = 0,
	}
};
////////////
typedef struct cmd {
	unsigned int regAddr;	///> 目标寄存器地址
	unsigned char clrCMD;	///> 命令内容(零位)
	unsigned char setCMD;	///> 命令内容(置位)
	unsigned char ret;		///> 操作结果
}cmd_t;

#define DEVICE_NAME		"tw2837"

typedef enum {
	ch_0 = 0,
	ch_1,
	ch_2,
	ch_3
} tw2837_ch;

struct tw2837_device_info {
    //  char dev 部分
    struct cdev*    dev_cdev; // 设备对象
    // i2c dev 专用部分
	struct i2c_client		*dev_client;		///>必须，操作系统提供的 client 对象指针
	
    // 根据需要
    unsigned int			pin_int;		///> 标记中断输入引脚编号
	unsigned int			pin_reset;		///> 标记复位控制输出引脚编号
};

static struct tw2837_device_info* tw2837_dev;
static struct class*   dev_class;           // 设备类对象
static int dev_major;                       // 主设备号
static int dev_minor;                       // 次设备号



/////////////////////
// 模块内部操作

/**
 * @brief 读 iic 总线数据
 * @param unsigned int addr 读内存的首地址
 * @param unsigned int len	读内存的长度
 * @param unsigned char  *data	数据缓冲区指针
*/
static int tw2837_read_block(unsigned int reg,unsigned int len,unsigned char  *data)
{
	unsigned char msgbuf0[2] = {0};
	unsigned int slave = tw2837_dev->dev_client->addr;
	unsigned int flags = tw2837_dev->dev_client->flags;

	// 准备两条消息：
	// msg_1: start condition + slave addr(LSB=1,write) + cmd
	// msg_2: start condition + slave addr(LSB=0,read) + Receive Data0 + Receive Data1 + Stop condition
	struct i2c_msg msg[2] = {
		{ slave, flags				, 2		, msgbuf0 	},
		{ slave, flags | I2C_M_RD	, len	, data 		}
	};

	msgbuf0[0] = (reg & 0xFF00 ) >> 8;
	msgbuf0[1] = reg & 0x00FF;
	// 启动发送
	return i2c_transfer(tw2837_dev->dev_client->adapter, msg, ARRAY_SIZE(msg));
}

/**
 * @brief 写 iic 总线数据
 * @param unsigned int addr 写内存的首地址
 * @param unsigned int len	写内存的长度
 * @param unsigned char  *data	数据缓冲区指针
*/
static int tw2837_write_block(unsigned int reg, unsigned int len, unsigned char  *data)
{
	unsigned char msgbuf[3] = {0};
	struct i2c_msg msg;

	msg.addr = tw2837_dev->dev_client->addr;
	msg.flags = tw2837_dev->dev_client->flags;
	msg.len = 3;
	msg.buf = msgbuf;

	msgbuf[0] = (reg & 0xFF00 ) >> 8;
	msgbuf[1] = reg & 0x00FF;
	msgbuf[2] = *(data+0);

	// 启动发送
	return i2c_transfer(tw2837_dev->dev_client->adapter,&msg,1);
}

// test
static int tw2837_writeCmd(cmd_t* currentCmd)
{
	unsigned char readBuff = 0 ;
	unsigned char tempCmd  = 0 ;
	// 先读
	if (tw2837_read_block(currentCmd->regAddr,1,&readBuff) < 0 ){
		dev_err( &tw2837_dev->dev_client->dev,"[E] TW2823: Reading failed:reg=0x%04x\n",currentCmd->regAddr);							
	}

	// 执行“与清零”和“或置位”
	tempCmd = readBuff & (~currentCmd->clrCMD) ;
	tempCmd |= currentCmd->setCMD ;

	// 写回
	if (tw2837_write_block(	currentCmd->regAddr,1,&tempCmd )<0){
		dev_err( &tw2837_dev->dev_client->dev,"[E] TW2823: Writing failed:reg=0x%04x\n",currentCmd->regAddr);
		return -1;
	}

	// 复读校验
	//printk(KERN_DEBUG "check cmd @%d\n",currentCmd->regAddr);
	if (tw2837_read_block(currentCmd->regAddr,1,&readBuff) < 0 ){
		//printk(KERN_DEBUG "read fail\n");
		dev_err( &tw2837_dev->dev_client->dev,"[E] TW2823: Reading failed:reg=0x%04x\n",currentCmd->regAddr);							
	} else {
		if ( readBuff != tempCmd ){
			// 校验失败
			printk(KERN_DEBUG "[E] TW2823: check fail @0x%04x,set=0x%02x,get=0x%02x\n",currentCmd->regAddr,tempCmd,readBuff);
		} else {
			// OK
			printk(KERN_DEBUG "[TW2837] @0x%04x,set=0x%02x\n",currentCmd->regAddr,tempCmd);
		}
	}
	return 0;
}


// static int tw2837_read_cmd(cmd_t* currentCmd)
// {
// 	unsigned char readBuff = 0 ;

// 	//printk(KERN_DEBUG "check cmd @%d\n",currentCmd->regAddr);
// 	if (tw2837_read_block(currentCmd->regAddr,1,&readBuff) < 0 ){
// 		printk(KERN_DEBUG "[E] TW2823: Reading failed:reg=0x%04x\n",currentCmd->regAddr);
// 		dev_err( &tw2837_dev->dev_client->dev,"[E] Reading failed:reg=0x%04x\n",currentCmd->regAddr);							
// 	} else {
// 		printk(KERN_INFO "[TW2837] @0x%04x,0x%02x\n",currentCmd->regAddr,readBuff);	
// 	}
// 	return 0;
// }

/**
 * @brief 复位 tw2823
 * 通过控制 gpio 端口产生 10ms 的低电平信号复位芯片
*/
static int tw2837_reset(void)
{
	// 硬件端口复位
	gpio_set_value(tw2837_dev->pin_reset, 0);
	msleep(10);
	gpio_set_value(tw2837_dev->pin_reset, 1);
	
	printk(KERN_DEBUG "TW2823 Reset completed\n");
	return 0;
}


static int tw2837_request_pins(struct device_node *node,unsigned int *reset_pin)
{
	unsigned int pin_rst = 0;

	if (of_gpio_count(node) < 1)
		return -ENODEV;

	// dts 中描述的顺序决定以下函数的第二个参数，比如：在 dts 中，
	// rst 首先描述，则应该采用 of_get_gpio(np, 0) 获取 int pin 的资源
	pin_rst = of_get_gpio(node, 0);

	*reset_pin = pin_rst;

	tw2837_dev->pin_reset = pin_rst;

	// 检查该 IO 是否可用，是否与其他模块存在冲突
	if (!gpio_is_valid(pin_rst)) {
		printk(KERN_DEBUG "[E] TW2823: Reset pin invalid\n");
		return -ENODEV;
	}
	printk(KERN_DEBUG "[TW2837] Reset pin valid\n");


	// 请求 interrupt 端口资源,输入
	/*
	由 gpio_request(unsigned gpio, const char *label) 
	调用 gpio_to_desc(gpio)，将 IO number 转为“描述符”，如果可用，则通过 gpiod_request(desc, label)获取IO
	*/
	// 请求 reset 端口资源，输出
	if ((gpio_request(pin_rst, "pin_reset") == 0) && (gpio_direction_output(pin_rst, 1) == 0)) {
		tw2837_dev->pin_reset = pin_rst;
		
	} else {
		printk(KERN_DEBUG "[E] TW2823: gpio_request failed\n");
		gpio_free(pin_rst); // 释放 pin_int 的 IO 资源
		return -ENODEV;
	}
	printk(KERN_DEBUG "[TW2837] gpio_request OK\n");
	return 0;
}

static int tw2837_release_pins(void)
{
	printk(KERN_DEBUG "[TW2837] release_pins\n");
	gpio_free(tw2837_dev->pin_reset); // 释放 pin_int 的 IO 资源
	return 0;
}


/*
//柳工8寸样机原理图设计错误，导致Vin1 的 A 和 B 交叉，需要设置 Vin1_B 为输入
{.regAddr=0x001D,.clrCMD=0xFB,.setCMD=0x04,.ret=0},
{.regAddr=0x001D,.clrCMD=0xF3,.setCMD=0x0C,.ret=0}, // 特殊处理，后续 PCB 更正后，恢复默认设置（VIN_1_A=video input）
*/
static int tw2837_set_common(void)
{
	int i   = 0;
	cmd_t cmdSequence[] = {
		// basic config for all channel

		// 0x01A4 配置必须为主模式，否则不能输出
		/*
		b7=1,master
		b6=0,bt656 output data range NOT limited
		b5=0,vertical sync detection from VSENC pin
		b4=0,field polarity detection from FLDENC pin

		b3=0,field polarity of ITU-R BT 656 output,High for even field
		b2=0,horizontal sync polarity,Active low
		b1=0,vertical sync polarity,Active low
		b0=0,field polarity,Even field is high
		*/ 
		{.regAddr=0x01A4,.clrCMD=0xFF,.setCMD=0x80,.ret=0},	// ENC_MODE = MASTER, Must!!

		// clock delay output of CLK54MEM pin
		/*
		b7=0,reserved
		b[6..4]=0,relative data delay for cascade channel extension
		b[3..0]=32,the clock delay of the CLK54MEM pin
		*/
		{.regAddr=0x017E,.clrCMD=0xFF,.setCMD=0x88,.ret=0},	//88, MCLKDEL=8

		// select video input for channel
		{.regAddr=0x0080,.clrCMD=0xFF,.setCMD=0x00,.ret=0},	// CH0=VIN0
		{.regAddr=0x0090,.clrCMD=0xfF,.setCMD=0x40,.ret=0}, // CH1=VIN1
		{.regAddr=0x00A0,.clrCMD=0xfF,.setCMD=0x80,.ret=0}, // CH2=VIN2
		{.regAddr=0x00B0,.clrCMD=0xfF,.setCMD=0xC0,.ret=0},	// CH3=VIN3 
		
		// disable all channel
		/*
		b7=0,disable channel
		b6=0,disable pop_up
		b5=0,operation mode ,live mode
		b4=0,switching path on PB display mode with PB_AUTO_EN = 1,Main channel selection

		b3=0,input between Live and PB for each channel,Normal live analog input
		b[2..0]=0,1,2,3
		*/
		{.regAddr=0x0110,.clrCMD=0xFF,.setCMD=0x00,.ret=0}, // CH0 TURN OFF
		{.regAddr=0x0118,.clrCMD=0xFf,.setCMD=0x01,.ret=0}, // CH1 TURN OFF
		{.regAddr=0x0120,.clrCMD=0xFf,.setCMD=0x02,.ret=0}, // CH2 TURN OFF
		{.regAddr=0x0128,.clrCMD=0xFf,.setCMD=0x03,.ret=0}, // CH3 TURN OFF

		// add config,after found the error of video
		// video ADC turn on 
		// audio ADC and DAC turn off
		/*
		b[7..6]=00,reserved
		b5=1,audio DAC power down
		b4=1,audio ADC power down
		b[3..0],ch3..ch0,video ADC power down,0x0F=关闭视频输入，蓝屏
		*/
		{.regAddr=0x004C,.clrCMD=0xFF,.setCMD=0x30,.ret=0},

		// video output DAC turn off 
		/*
		b7=1,VAOCX DAC power down,
		b6=0,reserved
		b[5..4]=00,analog video format for VAOYX DAC,no output

		b3=1,VAOYX DAC power down
		b2=0,reserved
		b[1..0]=00,analog video format for VAOCX DAC,no output
		*/
		{.regAddr=0x01A1,.clrCMD=0xFF,.setCMD=0x88,.ret=0},
		
		// turn off DAOYY DAC
		// VAOYY DAC=power down,DAC_OUT_YY=no output
		/*
		b7=0,reserved
		b[6..4]=00,analog video format for VAOYY DAC,No output 
		b3=0,power down of VAOYY DAC,power down of DAC
		b[2..0]=000,reserved
		*/
		{.regAddr=0x01A2,.clrCMD=0xFF,.setCMD=0x08,.ret=0},

		// VDOX digital video output
		/* the video data for analog output of video encoder
		 * b[7..6]=00,Video data of display path without OSD and mouse overlay
		 * b[5..4]=00
		 * the video data for ITU-R BT 656 digital output
		 * b[3..2]=00 Video data of display path without OSD and mouse overlay
		 * b[1..0]=00
		 * */ 
	    {.regAddr=0x01A0,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

		/*
		 * b7=0,the digital data output format,BT.656 mode
		 * b6=0,reserved
		 * b[5..4]=00,CCIR_OUT_X,the mode of ITU-R BT.656 digital output,Display path video data with single output mode (27MHz)
		 * b3=0,Swap Y/C output port when CCIR_601 = 1,
		 * b2=0,reserved
		 * b[1..0]=00,CCIR_OUT_Y,the mode of ITU-R BT.656 digital output,Display path video data with single output mode (27MHz)
		 * */
		{.regAddr=0x01A3,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

		/*
		 * b7=1,the operation mode of video encoder,MASTER,必须是 master
		 * b6=1,the data range of ITU-R BT 656 output,1 ~ 254,貌似更清晰
		 * b5=0,the vertical sync detection type
		 * b4=0,the field polarity detection type
		 *
		 * b3=0,the field polarity of ITU-R BT 656 output,High for even field
		 * b2=0,the horizontal sync polarity.Active low
		 * b1=0,the vertical sync polarity,Active Low
		 * b0=0,the field polarity,even field is high
		 * */
		{.regAddr=0x01A4,.clrCMD=0xFF,.setCMD=0xC0,.ret=0},
                /*
		 * b[7..6]=0~3,the field offset for first active video line
		 * b[5..0]=0~63,the line delay of vertical sync from active video by 1 line/step
		 * */
		{.regAddr=0x01A5,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

		/*
		b[7..0] Control the pixel delay of horizontal sync from active video by 1/2 pixel/step[9..2],def=128,0xFE
		*/
		{.regAddr=0x01A6,.clrCMD=0xFF,.setCMD=0x3F,.ret=0},// 3F

		/*
		b[7..6] Control the pixel delay of horizontal sync from active video by 1/2 pixel/step[1..0]
		b5=0,reserved
		b[4..0],Control the line delay of active video by 1 line/step,def=12,0 line delayed
		*/
		{.regAddr=0x01A7,.clrCMD=0xFF,.setCMD=0x0C,.ret=0},

		/*
		b7=0,the active delay mode for digital BT. 656 output,=both analog encoder and digital output
		b6=1,ITU-R BT656 standard format for 60Hz system,=244 line for odd and 243 line for even field (ITU-R BT.656 standard)
		b[5..0],the pixel delay of active video by 1 pixel/step,def=32
		
		b6=1,单场可以看到水平消音区域，另一场不会看到消音
		b[5..0]，可以改变可视范围宽度
		*/
		{.regAddr=0x01A8,.clrCMD=0xFF,.setCMD=0x60,.ret=0}, // 60

		/*
		b[7..6],Set color sub-carrier frequency for video encoder
		b[5..3]=001
		b2,Set the phase alternation of encoder
		b1,Reset the phase alternation every 8 field
		b0,Set 7.5IRE for pedestal level
		不影响滚动
		*/
		{.regAddr=0x01A9,.clrCMD=0xFF,.setCMD=0xCA,.ret=0},//1100 1000

		/*
		b[7..6],X,Control the chrominance bandwidth of video encoder. def=10
		b[5..4],X,Control the luminance bandwidth of video encoder ,def=10
		b[3..2],Y,Control the chrominance bandwidth of video encoder,def=10
		b[1..0],Y,Control the luminance bandwidth of video encoder,def=10
		*/ 
		{.regAddr=0x01AA,.clrCMD=0xFF,.setCMD=0xAA,.ret=0}, // 10101010

		// test mode，没有测试条纹
		/*
		b3,b1,Enable the test pattern output.
		b2,b0,Enable the color killing function
		*/
		{.regAddr=0x01AB,.clrCMD=0xFF,.setCMD=0x05,.ret=0},
		
		/*
		b[7..6],Control the clock frequency of CLKVDOX pin (default = 1, 27MHz),01
		b[5..4],Control the clock phase of CLKVDOX pin (default = 0, 0 degree),0(0 deg) or 2(180 deg)
		b[3..0],Control the clock delay of CLKVDOX pin,0 or 32
		不影响
		*/
		{.regAddr=0x01AC,.clrCMD=0xFF,.setCMD=0x40,.ret=0},


		// color
		//{.regAddr=0x010F,.clrCMD=0xFF,.setCMD=0x2A,.ret=0},

		/*
		b[7..4],read only
		b3=1,dis shadow registers
		b[2..0]=7,auto detection
		*/		
		{.regAddr=0x000E,.clrCMD=0xFF,.setCMD=0x0F,.ret=0},

		/*
		b7,sys_5060,=1,50Hz,625
		b6,overlay,=0,disable overlay
		b5,link_last,=1,lowest slave chip
		b4,link_en,=1,enable cascade operation
		b3,link_num,=0,master
		*/
		{.regAddr=0x0100,.clrCMD=0xFF,.setCMD=0x80,.ret=0},
		
		// 以下配置 SDRAM，默认内容为0,属于不可用的数值范围，必须经过设置
		// 这个比较奇怪，默认值居然是不能用的 !!!!,至少有个默认值可用啊
		// Save addr of SDRAM,4~11 for 64M SDRAM
		{.regAddr=0x0102,.clrCMD=0xFF,.setCMD=0x08,.ret=0},
		// Recall address for main channel
		{.regAddr=0x0112,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
		{.regAddr=0x011A,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
		{.regAddr=0x0122,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
		{.regAddr=0x012A,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
		// end add
		
		// init memory
		{.regAddr=0x017F,.clrCMD=0xFF,.setCMD=0x80,.ret=0},

		// 背景　background color=blue
		{.regAddr=0x010F,.clrCMD=0xFF,.setCMD=0x3C,.ret=0},

		{.regAddr=0x0105,.clrCMD=0xFF,.setCMD=0x04,.ret=0},
	};
	
	for (i=0;i<ARRAY_SIZE(cmdSequence);i++) {
		tw2837_writeCmd(&cmdSequence[i]);
	}
	return 0;
}

static int tw2837_set_magic_regs(void)
{
	int i=0;
	cmd_t cmdSequence[] = {
		// 寄存器表给定的默认值
		{.regAddr=0x0043,.clrCMD=0xFF,.setCMD=0xC5,.ret=0}, // regs map,set value
		{.regAddr=0x0044,.clrCMD=0xFF,.setCMD=0xA1,.ret=0}, // regs map,set value
		{.regAddr=0x004E,.clrCMD=0xFF,.setCMD=0x05,.ret=0}, // regs map,set value

		{.regAddr=0x0053,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// reserved
		{.regAddr=0x0054,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// reserved,recommended

		{.regAddr=0x00F6,.clrCMD=0xFF,.setCMD=0x08,.ret=0},
		{.regAddr=0x00F7,.clrCMD=0xFF,.setCMD=0x08,.ret=0},
		{.regAddr=0x00F8,.clrCMD=0xFF,.setCMD=0x08,.ret=0},
		{.regAddr=0x00F9,.clrCMD=0xFF,.setCMD=0x08,.ret=0},

		// HFLT1~4,没有说明
		{.regAddr=0x0050,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x0051,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//

		// 明确说明保留，正常模式下必须设置为如下状态 
		{.regAddr=0x00C7,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},//
		{.regAddr=0x00C9,.clrCMD=0xFF,.setCMD=0x3C,.ret=0},//

		{.regAddr=0x0117,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x011F,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x0127,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x012F,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//

		{.regAddr=0x0153,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x017D,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//

		{.regAddr=0x01B7,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01B8,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01B9,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01BA,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01BB,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01BC,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01BD,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01BE,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
		{.regAddr=0x01BF,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//

		{.regAddr=0x00F4,.clrCMD=0xFF,.setCMD=0x00,.ret=0},//
	};

	for (i=0;i<ARRAY_SIZE(cmdSequence);i++) {
		tw2837_writeCmd(&cmdSequence[i]);
	}
	return 0;
}



static int tw2837_set_delay_active( tw2837_ch ch )
{
	int i=0;
	cmd_t cmdSequence[] = {
		// HDELAY
		{.regAddr=0x0002,.clrCMD=0xFF,.setCMD=0x01,.ret=0}, // HDELAY 0X09=9,黑边减少明显
		{.regAddr=0x0006,.clrCMD=0x03,.setCMD=0x00,.ret=0},
		
		// HACTIVE,0X2D0=720,如果设置为0x0D0（２０８），则画面只有右侧部分
		{.regAddr=0x0003,.clrCMD=0xFF,.setCMD=0xCD,.ret=0}, // HACTIVE,0X2D0=720
		{.regAddr=0x0006,.clrCMD=0x0C,.setCMD=0x08,.ret=0}, // 
		// VDELAY 0xff(256),只显示底部的部分
		{.regAddr=0x0004,.clrCMD=0xFF,.setCMD=0x06,.ret=0}, // VDELAY 0X01=1,def=06
		{.regAddr=0x0006,.clrCMD=0x10,.setCMD=0x00,.ret=0},
		// VACTIVE,def=0x120(288),
		{.regAddr=0x0005,.clrCMD=0xFF,.setCMD=0x20,.ret=0}, // 0x20,VACTIVE 0X120=288
		{.regAddr=0x0006,.clrCMD=0x20,.setCMD=0x20,.ret=0}, // 0x20
	};

	cmdSequence[0].setCMD = (h_delay & 0x00FF);
	cmdSequence[1].setCMD = (h_delay & 0x0300) >> 8 ;

	cmdSequence[2].setCMD = (h_active & 0x00FF);
	cmdSequence[3].setCMD = (h_active & 0x0300) >> 6 ;

	cmdSequence[4].setCMD = (v_delay & 0x00FF);
	cmdSequence[5].setCMD = (v_delay & 0x0100) >> 4 ;

	cmdSequence[6].setCMD = (v_active & 0x00FF);
	cmdSequence[7].setCMD = (v_active & 0x0100) >> 3 ;

	for (i=0;i<ARRAY_SIZE(cmdSequence);i++) {
		cmdSequence[i].regAddr |= (ch<<4);
		tw2837_writeCmd(&cmdSequence[i]);
	}
	return 0;
	
}

static int tw2837_set_property( const channel_property_t prop )
{
	int i=0;
	cmd_t cmdSequence[] = {
		// HDELAY
		{.regAddr=0x0002,.clrCMD=0xFF,.setCMD=0x01,.ret=0}, // HDELAY 0X09=9,黑边减少明显
		{.regAddr=0x0006,.clrCMD=0x03,.setCMD=0x00,.ret=0},
		
		// HACTIVE,0X2D0=720,如果设置为0x0D0（２０８），则画面只有右侧部分
		{.regAddr=0x0003,.clrCMD=0xFF,.setCMD=0xCD,.ret=0}, // HACTIVE,0X2D0=720
		{.regAddr=0x0006,.clrCMD=0x0C,.setCMD=0x08,.ret=0}, // 
		// VDELAY 0xff(256),只显示底部的部分
		{.regAddr=0x0004,.clrCMD=0xFF,.setCMD=0x06,.ret=0}, // VDELAY 0X01=1,def=06
		{.regAddr=0x0006,.clrCMD=0x10,.setCMD=0x00,.ret=0},
		// VACTIVE,def=0x120(288),
		{.regAddr=0x0005,.clrCMD=0xFF,.setCMD=0x20,.ret=0}, // 0x20,VACTIVE 0X120=288
		{.regAddr=0x0006,.clrCMD=0x20,.setCMD=0x20,.ret=0}, // 0x20
	};

	cmdSequence[0].setCMD = (prop.h_delay & 0x00FF);
	cmdSequence[1].setCMD = (prop.h_delay & 0x0300) >> 8 ;

	cmdSequence[2].setCMD = (prop.h_active & 0x00FF);
	cmdSequence[3].setCMD = (prop.h_active & 0x0300) >> 6 ;

	cmdSequence[4].setCMD = (prop.v_delay & 0x00FF);
	cmdSequence[5].setCMD = (prop.v_delay & 0x0100) >> 4 ;

	cmdSequence[6].setCMD = (prop.v_active & 0x00FF);
	cmdSequence[7].setCMD = (prop.v_active & 0x0100) >> 3 ;

	for (i=0;i<ARRAY_SIZE(cmdSequence);i++) {
		cmdSequence[i].regAddr |= (prop.ch << 4);
		tw2837_writeCmd(&cmdSequence[i]);
	}
	return 0;
	
}

static int tw2837_set_on(tw2837_ch videoCH,const unsigned int on)
{
	cmd_t cmdSequence;
	unsigned int temp = (unsigned int)videoCH ;

	cmdSequence.clrCMD = 0xFF ;
	cmdSequence.setCMD = 0x00 ;
	if ( on ){
		cmdSequence.setCMD = temp | 0x80 ;
	}
	cmdSequence.regAddr = 0x0110 + (8*(unsigned int)videoCH);
	tw2837_writeCmd(&cmdSequence);
	return 0;
}

static int tw2837_set_scale_position(	tw2837_ch videoCH,const unsigned int x,const unsigned int y,const unsigned int width,const unsigned int height)
{
	int i   = 0;
	unsigned int hScaleRatio = 0;
	unsigned int vScaleRatio = 0;

	unsigned int picHL = 0;
	unsigned int picHR = 0;

	unsigned int picVT = 0;
	unsigned int picVB = 0;

	unsigned int ch = (unsigned int)videoCH;

	cmd_t cmdSequence[] = {
		{.regAddr=0x0081,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [15:8]
		{.regAddr=0x0082,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [7:0]
		{.regAddr=0x0083,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [15:8]
		{.regAddr=0x0084,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [7:0]
		{.regAddr=0x0130,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picHL
		{.regAddr=0x0131,.clrCMD=0xFF,.setCMD=0xB4,.ret=0},// picHR
		{.regAddr=0x0132,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picVT
		{.regAddr=0x0133,.clrCMD=0xFF,.setCMD=0x90,.ret=0},// picVB
	};

	picHL = (unsigned int)(( x *180)/100);
	picHR = (unsigned int)(((x + width)*180)/100);
	printk( KERN_DEBUG "[TW2837] picHL=%i , picHR=%i\n",picHL,picHR);

	picVT = (unsigned int)(( y *144)/100);
	picVB = (unsigned int)(((y + height)*144)/100);

	hScaleRatio = (unsigned int)((width * 0xFFFF)/100);
	vScaleRatio = (unsigned int)((height * 0xFFFF)/100);
	printk( KERN_DEBUG "[TW2837] hScaleRatio=%i , vScaleRatio=%i\n",hScaleRatio,vScaleRatio);

	cmdSequence[0].regAddr |= (ch<<4);
	cmdSequence[0].setCMD = ( vScaleRatio & 0xFF00 ) >> 8 ;
	cmdSequence[1].regAddr |= (ch<<4);
	cmdSequence[1].setCMD = vScaleRatio & 0x00FF;

	cmdSequence[2].regAddr |= (ch<<4);
	cmdSequence[2].setCMD = ( hScaleRatio & 0xFF00 ) >> 8 ;
	cmdSequence[3].regAddr |= (ch<<4);
	cmdSequence[3].setCMD = hScaleRatio & 0x00FF;

	cmdSequence[4].regAddr |= (ch<<2);
	cmdSequence[4].setCMD = picHL;
	cmdSequence[5].regAddr |= (ch<<2);
	cmdSequence[5].setCMD = picHR;

	cmdSequence[6].regAddr |= (ch<<2);
	cmdSequence[6].setCMD = picVT;
	cmdSequence[7].regAddr |= (ch<<2);
	cmdSequence[7].setCMD = picVB;

	for (i=0;i<ARRAY_SIZE(cmdSequence);i++) {
		tw2837_writeCmd(&cmdSequence[i]);
	}
	return 0;
}

// static int tw2837_setVideoWindow(char* cmdBuf)
// {
// 	char topDelim[] = "/";
// 	char subDelim[] = ",";	
// 	char* subCMD[5];
// 	char* ssubCMD[5][7];
//     char *token;  
// 	unsigned int i = 0;
// 	unsigned int j = 0;

// 	unsigned int channelCount = 0;
// 	unsigned int ch = 0,x,y,w,h;

// 	// 第一层分割通道;第二层分割通道内的属性
// 	for(token = strsep(&cmdBuf, topDelim); token != NULL; token = strsep(&cmdBuf, topDelim)) {  
//         subCMD[i] = token;
// 		j = 0;
// 		for(token = strsep(&subCMD[i], subDelim); token != NULL; token = strsep(&subCMD[i],subDelim)) {  
// 			ssubCMD[i][j] = token;
// 			j++;
// 		}
// 		i++;
// 		if ( i>4 ) {
// 			goto err;
// 		}
//     }

// 	channelCount = i;

// 	for ( i=0;i<channelCount;i++){
// 		ch = simple_strtoul(ssubCMD[i][0],NULL,10);
// 		if ( strcmp(ssubCMD[i][1],"off")==0 ){
// 			// turn off
// 			tw2837_set_scale_position((tw2837_ch)ch,0,0,0,0);
// 			tw2837_set_on((tw2837_ch)ch,0);
// 			continue;
// 		} else if( strcmp(ssubCMD[i][1],"on")==0 ){
// 			// turn on
// 			x = simple_strtoul(ssubCMD[i][2],NULL,10);
// 			y = simple_strtoul(ssubCMD[i][3],NULL,10);
// 			w = simple_strtoul(ssubCMD[i][4],NULL,10);
// 			h = simple_strtoul(ssubCMD[i][5],NULL,10);
// 			tw2837_set_scale_position((tw2837_ch)ch,x,y,w,h);
// 			tw2837_set_on((tw2837_ch)ch,1);
// 		}
// 	}
// 	return 0;
// err:
// 	return -1;
// }


// 结束 模块内部操作
/////////////////////

/*
struct v4l2_ifparm {
        enum v4l2_if_type if_type;
        union {
                struct v4l2_if_type_bt656 bt656;
        } u;
};

// Slave interface type. 
enum v4l2_if_type {
        //Parallel 8-, 10- or 12-bit interface, used by for example
        //on certain image sensors.
        V4L2_IF_TYPE_BT656,
		
};

struct v4l2_if_type_bt656 {
        // /
        // 0: Frame begins when vsync is high.
        // 1: Frame begins when vsync changes from low to high.
        //  
        unsigned frame_start_on_rising_vs:1;
        // / Use Bt synchronisation codes for sync correction.
        unsigned bt_sync_correct:1;
        // / Swap every two adjacent image data elements.
        unsigned swap:1;
        // / Inverted latch clock polarity from slave. 
        unsigned latch_clk_inv:1;
        // / Hs polarity. 0 is active high, 1 active low.
        unsigned nobt_hs_inv:1;
        // / Vs polarity. 0 is active high, 1 active low
        unsigned nobt_vs_inv:1;
        enum v4l2_if_type_bt656_mode mode;
        // / Minimum accepted bus clock for slave (in Hz).
        u32 clock_min;
        // / Maximum accepted bus clock for slave.
        u32 clock_max;
        // /
        // Current wish of the slave. May only change in response to
        // ioctls that affect image capture.
        // 
        u32 clock_curr;
};

enum v4l2_if_type_bt656_mode {
        /
        Modes without Bt synchronisation codes. Separate
        synchronisation signal lines are used.
        //
        V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT,
        V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT,
        V4L2_IF_TYPE_BT656_MODE_NOBT_12BIT,
        /
        Use Bt synchronisation codes. The vertical and horizontal
        synchronisation is done based on synchronisation codes.
        //
        V4L2_IF_TYPE_BT656_MODE_BT_8BIT,
        V4L2_IF_TYPE_BT656_MODE_BT_10BIT,
};
*/

// 启动　ｇｓｔ　过程执行
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct device *dev = &tw2837_data.sen.i2c_client->dev;

	printk(KERN_DEBUG "ioctl_g_ifparm\n");

	dev_dbg(dev, "adv7280: %s\n", __func__);

	if (s == NULL) {
		dev_err(dev, "  ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
#if 0
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.nobt_hs_inv = 1;
	p->u.bt656.bt_sync_correct = 1;
#else
	// p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_BT_8BIT;// yj，不使用独立的vs,hs
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT; 
	// p->u.bt656.nobt_hs_inv = 0;

	// 0: v4l2 capture driver to use internal VSYNC mode (yj)
	p->u.bt656.bt_sync_correct = 0;
	
	// yj
	p->u.bt656.clock_min=22000000;
	p->u.bt656.clock_max=27000000;
	// p->u.bt656.latch_clk_inv = 1;
	
	// low active
	// p->u.bt656.nobt_hs_inv = 1;
	// p->u.bt656.nobt_vs_inv = 1;
	
	// p->u.bt656.frame_start_on_rising_vs = 1;
	// end

	// clock_curr (yj)
	// 0: for BT.656/1120 interlaced clock mode
	// 1: de-interlace clock mode
#ifdef HW_DEINT
	p->u.bt656.clock_curr = 0;  /* BT656 de-interlace clock mode */
#endif

#endif
	/* ADV7280 has a dedicated clock so no clock settings needed. */

	return 0;
}

// 加载驱动过程执行一次
// 启动　ｇｓｔ　过程执行
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	printk(KERN_DEBUG "ioctl_s_power\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	// if (on && !sensor->sen.on) {
	// 	if (adv7280_write_reg(ADV7280_POWER_MANAGEMENT, 0x04) != 0)
	// 		return -EIO;

	// 	msleep(400);

	// } else if (!on && sensor->sen.on) {
	// 	if (adv7280_write_reg(ADV7280_POWER_MANAGEMENT, 0x24) != 0)
	// 		return -EIO;
	// }

	sensor->sen.on = on;

	return 0;
}


/*
struct v4l2_captureparm {
        __u32              capability;    Supported modes
        __u32              capturemode;   Current mode
        struct v4l2_fract  timeperframe;  Time per frame in seconds
        __u32              extendedmode;  Driver-specific extensions
        __u32              readbuffers;   # of buffers for read
        __u32              reserved[4];
};
*/

// 启动　ｇｓｔ　过程执行
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	struct device *dev = &tw2837_data.sen.i2c_client->dev;

	printk(KERN_DEBUG "ioctl_g_parm\n");

	dev_dbg(dev, "adv7280: %s\n", __func__);

	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_dbg(dev, "   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;
		

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		dev_dbg(dev, "ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

// 启动　ｇｓｔ　过程执行
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct device *dev = &tw2837_data.sen.i2c_client->dev;
	printk(KERN_DEBUG "ioctl_s_parm\n");

	dev_dbg(dev, "adv7280: %s\n", __func__);

	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		dev_dbg(dev, "   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}



static void tw2837_get_std(v4l2_std_id *std)
{
	int standard, idx;//status_1, 
	bool locked = 0;
	unsigned char readBuff = 0 ;

	printk(KERN_DEBUG "tw2837_get_std()\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "In %s\n", __func__);


	if (tw2837_read_block(0x0001,1,&readBuff) < 0 ){
		// printk(KERN_DEBUG "[E] TW2823: Reading failed:reg=0x%04x\n",currentCmd->regAddr);
	} else {
		// printk(KERN_INFO "[TW2837] @0x%04x,0x%02x\n",currentCmd->regAddr,readBuff);	
	}
	locked = readBuff & 0x08;
	printk(KERN_DEBUG "interlace:%i\n",locked);

	//printk(KERN_DEBUG "check cmd @%d\n",currentCmd->regAddr);
	if (tw2837_read_block(0x000E,1,&readBuff) < 0 ){
		// printk(KERN_DEBUG "[E] TW2823: Reading failed:reg=0x%04x\n",currentCmd->regAddr);
	} else {
		// printk(KERN_INFO "[TW2837] @0x%04x,0x%02x\n",currentCmd->regAddr,readBuff);	
	}
	locked = readBuff & 0x80;  // 1,detecting; 0,idle
	standard = (readBuff & 0x70) >> 4;

	mutex_lock(&mutex);
	*std = V4L2_STD_ALL;
	idx = TW2837_NOT_LOCKED;
	if ( !locked ) {
		switch ( standard ){
			case 0: // NTSC(M)
			case 3: // NTSC(4.43)
				*std = V4L2_STD_NTSC;
				idx = TW2837_NTSC;
				printk(KERN_DEBUG "V4L2_STD_NTSC\n");
			break;
			case 1: // PAL(B,D,G,H,I)
			case 4: // PAL(M)
			case 5: // PAL(CN)
			case 6: // PAL(60)
				*std = V4L2_STD_PAL	;//V4L2_STD_PAL_60 ;
				idx = TW2837_PAL;
				printk(KERN_DEBUG "V4L2_STD_PAL_60 : %d\n",standard);
			break;
		}
		// if (standard == TW2837_STATUS1_AUTOD_PAL_B_G) {
		// 	*std = V4L2_STD_PAL_60 ;//V4L2_STD_PAL;
		// 	idx = TW2837_PAL;
		// } else if (standard == TW2837_STATUS1_AUTOD_NTSM_M_J) {
		// 	*std = V4L2_STD_NTSC;
		// 	idx = TW2837_NTSC;
		// }
	}
	mutex_unlock(&mutex);

	/* This assumes autodetect which this device uses. */
	if (*std != tw2837_data.std_id) {
		video_idx = idx;
		tw2837_data.std_id = *std;
		tw2837_data.sen.pix.width = video_fmts[video_idx].raw_width;
		tw2837_data.sen.pix.height = video_fmts[video_idx].raw_height;
	}
}

// 加载过程执行一次,
// 启动　ｇｓｔ　过程多次执行
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct device *dev = &tw2837_data.sen.i2c_client->dev;
	struct sensor *sensor = s->priv;
	v4l2_std_id std;

	printk(KERN_DEBUG "ioctl_g_fmt_cap\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	// tw2837_get_std(&std); // yj
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_dbg(dev, "   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);

		printk(KERN_DEBUG "Returning size of %dx%d\n",sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		// f->fmt.pix.pixelformat = (u32)std; //yj
		break;

	case V4L2_BUF_TYPE_PRIVATE:
		tw2837_get_std(&std);
		f->fmt.pix.pixelformat = (u32)std;
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

static int ioctl_queryctrl(struct v4l2_int_device *s,struct v4l2_queryctrl *qc)
{
	int i;

	printk(KERN_DEBUG "ioctl_queryctrl\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(tw2837_qctrl); i++)
		if (qc->id && qc->id == tw2837_qctrl[i].id) {
			*qc = tw2837_qctrl[i];
			return 0;
		}

	return -EINVAL;
}

static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;
	int sat = 0;

	printk(KERN_DEBUG "ioctl_g_ctrl\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		//tw2837_data.sen.brightness = adv7280_read(ADV7280_BRIGHTNESS);
		vc->value = tw2837_data.sen.brightness;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		vc->value = tw2837_data.sen.contrast;
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		//sat = adv7280_read(ADV7280_SD_SATURATION_CB);
		tw2837_data.sen.saturation = sat;
		vc->value = tw2837_data.sen.saturation;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		vc->value = tw2837_data.sen.hue;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		vc->value = tw2837_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		vc->value = tw2837_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		vc->value = tw2837_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   Default case\n");
		vc->value = 0;
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	u8 tmp;

	printk(KERN_DEBUG "ioctl_s_ctrl\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		//adv7280_write_reg(ADV7280_BRIGHTNESS, tmp);
		tw2837_data.sen.brightness = vc->value;
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		//adv7280_write_reg(ADV7280_SD_SATURATION_CB, tmp);
		//adv7280_write_reg(ADV7280_SD_SATURATION_CR, tmp);
		tw2837_data.sen.saturation = vc->value;
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	default:
		dev_dbg(&tw2837_data.sen.i2c_client->dev,
			"   Default case\n");
		retval = -EINVAL;
		break;
	}

	return retval;
}

// 启动　ｇｓｔ　过程执行
static int ioctl_enum_framesizes(struct v4l2_int_device *s,struct v4l2_frmsizeenum *fsize)
{
	printk(KERN_DEBUG "ioctl_enum_framesizes\n");

	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height  = video_fmts[video_idx].active_height;

	printk(KERN_DEBUG "ioctl_enum_framesizes %d,%d\n",fsize->discrete.width,fsize->discrete.height);
	return 0;
}

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,struct v4l2_frmivalenum *fival)
{
	struct video_fmt_t fmt;
	int i;

	printk(KERN_DEBUG "ioctl_enum_frameintervals\n");


	if (fival->index != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(video_fmts) - 1; i++) {
		fmt = video_fmts[i];
		if (fival->width  == fmt.active_width &&
		    fival->height == fmt.active_height) {
			fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
			fival->discrete.numerator = 1;
			fival->discrete.denominator = fmt.frame_rate;
			return 0;
		}
	}

	return -EINVAL;
}

static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	printk(KERN_DEBUG "ioctl_g_chip_ident\n");

	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"tw2837_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = 2837;//V4L2_IDENT_ADV7280;

	return 0;
}

// 启动　ｇｓｔ　过程执行
static int ioctl_init(struct v4l2_int_device *s)
{
	printk(KERN_DEBUG "ioctl_init\n");
	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	return 0;
}

// 启动　ｇｓｔ　过程执行
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	printk(KERN_DEBUG "ioctl_dev_init\n");

	dev_dbg(&tw2837_data.sen.i2c_client->dev, "adv7280: %s\n", __func__);

	return 0;
}

static struct v4l2_int_ioctl_desc tw2837_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
	{vidioc_int_init_num, (v4l2_int_ioctl_func*)ioctl_init},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*)ioctl_g_ctrl},
	{vidioc_int_g_chip_ident_num, (v4l2_int_ioctl_func *)ioctl_g_chip_ident},

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func*)ioctl_queryctrl},

	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)
				ioctl_enum_frameintervals},
};

static struct v4l2_int_slave tw2837_slave = {
	.ioctls = tw2837_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tw2837_ioctl_desc),
};

static struct v4l2_int_device tw2837_int_device = {
	.module = THIS_MODULE,
	.name = "tw2837",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tw2837_slave,
	},
};

// KERN_DEBUG

/////////////////////
// 模块标准接口
static ssize_t tw2837_read(	struct file* file,
							char __user* buf,
							size_t count,
							loff_t* ptr){
	// printk(KERN_DEBUG "[TW2837] tw2837_read\n");
	// tw2837_read_base_setting();

	return 0;
}

static ssize_t tw2837_write(struct file* file,const char __user* buf,size_t count,loff_t* ptr)
{

	// printk( KERN_DEBUG "[TW2837] tw2837_write\n");
	// char* cmdBuf ;

	// cmdBuf = (char*)kmalloc( 128,GFP_KERNEL);
	// if (!cmdBuf){
	// 	printk(KERN_DEBUG "[E] TW2823: tw2837_write,kmalloc...fail\n");
	// 	goto err;
	// }
	// memset(cmdBuf,0,128);
	// if ( copy_from_user(cmdBuf,buf,count) ){
	// 	printk(KERN_DEBUG "[E] TW2823: tw2837_write,copy_from_user...fail\n");
	// 	kfree(cmdBuf);
	// 	goto err;
	// }

	// //tw2837_setCommand(cmdBuf);
	// tw2837_setVideoWindow(cmdBuf);
	// kfree(cmdBuf);

// 	return count;
// err:
// 	return -1;
	return 0;
}

static long tw2837_ioctl(struct file* file,
						unsigned int cmd,
						unsigned long arg)
{
	int err = 0;
	// int retval = 0;
	// int count = 0;
	// char* args_buf ;

	channel_property_t ch_prop = {0};

	if (_IOC_TYPE(cmd) != TW2823_IOC_MAGIC) 
		return -ENOTTY;
	if (_IOC_NR(cmd) > TW2823_IOC_MAXNR) 
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) 
		return -EFAULT;

	switch(cmd) {
		case TW2823_IOC_RESET:
			printk( KERN_DEBUG "[TW2837] IOCTL..Reset\n");
		break;
        
		case TW2823_IOC_CH_PROPERTY:{
			if ( copy_from_user( &ch_prop,(channel_property_t*)arg,sizeof(channel_property_t)) ){
				printk(KERN_DEBUG "[E] TW2823: tw2837_ioctl,copy_from_user...fail\n");
				return -EFAULT;
			}
			printk(KERN_DEBUG "[TW2837] ioctl,ch=%i\n",ch_prop.ch);
			tw2837_set_property( ch_prop );
		}
		break;

		case TW2823_IOC_H_ACTIVE:{
			h_active = arg;
			printk( KERN_DEBUG "[TW2837] IOCTL..H active,%i\n",h_active);
			tw2837_set_delay_active(0);
			tw2837_set_delay_active(1);
			tw2837_set_delay_active(2);
			tw2837_set_delay_active(3);
		}
		break;

		case TW2823_IOC_V_DELAY:
			v_delay = arg;
			printk( KERN_DEBUG "[TW2837] IOCTL..V delay,%i\n",h_active);
			tw2837_set_delay_active(0);
			tw2837_set_delay_active(1);
			tw2837_set_delay_active(2);
			tw2837_set_delay_active(3);
		break;

		case TW2823_IOC_V_ACTIVE:
			v_active = arg;
			printk( KERN_DEBUG "[TW2837] IOCTL..V active,%i\n",h_active);
			tw2837_set_delay_active(0);
			tw2837_set_delay_active(1);
			tw2837_set_delay_active(2);
			tw2837_set_delay_active(3);
		break;


	  	case TW2823_IOC_SETWINDOW:
	  	// count = strlen((char*)arg) ; 
	  	// cmdBuf = (char*)kmalloc( 128,GFP_KERNEL);
		// if (!cmdBuf){
		// 	printk(KERN_DEBUG "[E] TW2823: tw2837_ioctl,kmalloc...fail\n");
		// 	goto err;
		// }
		// memset(cmdBuf,0,128);
		// if ( copy_from_user(cmdBuf,(char*)arg,count) ){
		// 	printk(KERN_DEBUG "[E] TW2823: tw2837_ioctl,copy_from_user...fail\n");
		// 	kfree(cmdBuf);
		// 	goto err;
		// }
	  	// tw2837_setVideoWindow(cmdBuf);
		// kfree(cmdBuf);
		break;
	  default:
		return -ENOTTY;
	}
	
// 	return retval;
// err:
// 	return -1;
	return 0;
}

static int tw2837_open(	struct inode* inode,
						struct file* file){
	// printk( KERN_DEBUG "[TW2837] tw2837_open\n");

	return 0;
}

static int tw2837_release(	struct inode* inode,
							struct file* file){
	// printk( KERN_DEBUG "[TW2837] tw2837_release\n");
	return 0;
}

struct file_operations tw2837_fops = {
	.owner   = 	THIS_MODULE,
	.read    = 	tw2837_read,
	.write 	 = 	tw2837_write,
	.open           = tw2837_open,
	.release        = tw2837_release,
	.unlocked_ioctl = tw2837_ioctl,
};
// 结束 模块标准接口
//////////////////////////////


/*
V4L2_PIX_FMT_UYVY 接收格式
Cb00 	Y’00 	Cr00 	Y’01 	Cb01 	Y’02 	Cr01 	Y’03

tw2837　输出格式
Cb0	Y0	Cr0	Y1	Cb2	Y2	Cr2	Y3

*/
/////////////////////////////
// 模块入口和出口
static int tw2837_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret = 0;
	// struct pinctrl *pinctrl;
	struct device *dev = &i2c->dev;

	struct device_node *dts_node = i2c->dev.of_node; // 获取设备树中描述的节点信息


    printk(KERN_DEBUG "[TW2837] driver probe...\n");

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "Required functionality not supported by I2C adapter\n");
		return -EIO;
	}

	v4l_info(i2c, "chip found @ 0x%02x (%s)\n",i2c->addr << 1, i2c->adapter->name);

	tw2837_dev->dev_client = i2c ;
	i2c_set_clientdata(i2c,tw2837_dev);

	dev_info(	&i2c->dev,
				"tw2837 (%d,0x%02x) registered (tw2837 V1.0.2)\n",
				i2c_adapter_id(tw2837_dev->dev_client->adapter),
				tw2837_dev->dev_client->addr);

	// 检查确认节点：i2c->dev.of_nodes
	if (dts_node != NULL) {
		// 根据 dts 中的描述，通过 of_node 申请 IO 资源
		if ( tw2837_request_pins(i2c->dev.of_node,&tw2837_dev->pin_reset) < 0 ){
			printk(KERN_DEBUG "[E] TW2823: Fail to request reset pin\n");
		} else {
			tw2837_reset();
			printk(KERN_DEBUG "[TW2837] low pluse on reset pin \n");
		}
	}


	memset(&tw2837_data, 0, sizeof(tw2837_data));
	tw2837_data.sen.i2c_client = i2c;

	//帧率，用分数表示
	tw2837_data.sen.streamcap.timeperframe.denominator = 25;
	tw2837_data.sen.streamcap.timeperframe.numerator = 1;

	// 2837返回的模拟信号格式：PAL_B/G/D
	tw2837_data.std_id = V4L2_STD_PAL;// yj V4L2_STD_ALL;
	video_idx = TW2837_PAL ;//yj,TW2837_NOT_LOCKED;
	
	// 720*625 包含消音区间
	tw2837_data.sen.pix.width = video_fmts[video_idx].raw_width;
	tw2837_data.sen.pix.height = video_fmts[video_idx].raw_height;

	// 与2837的输出一致
	tw2837_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;  /* YUV422 */
	
	tw2837_data.sen.pix.priv = 1;/* 1 is used to indicate TV in */
	tw2837_data.sen.on = true;

	printk(KERN_DEBUG "[TW2837] tw2837_data.sen.pix ( %d x %d) \n",tw2837_data.sen.pix.width,tw2837_data.sen.pix.height);

	ret = of_property_read_u32(dev->of_node, "csi_id",&(tw2837_data.sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	/* parallel  */
	tw2837_data.sen.mipi_camera = 0;

	clk_prepare_enable(tw2837_data.sen.sensor_clk);

	tw2837_set_magic_regs();
	tw2837_set_common();
	tw2837_set_delay_active(0);
	tw2837_set_delay_active(1);
	tw2837_set_delay_active(2);
	tw2837_set_delay_active(3);
	tw2837_set_scale_position(0,10,10,80,80);
	// tw2837_set_scale_position(1,49,50,49,50);
	tw2837_set_on(0,1);
	// tw2837_set_on(1,0);
	// tw2837_set_on(2,0);
	// tw2837_set_on(3,0);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the tw2837_data structure here.*/
	tw2837_int_device.priv = &tw2837_data;
	ret = v4l2_int_device_register(&tw2837_int_device);

	clk_disable_unprepare(tw2837_data.sen.sensor_clk);



    return 0;	
}

static int tw2837_remove(struct i2c_client *i2c)
{
	printk(KERN_DEBUG "[TW2837] tw2837_remove\n");
	tw2837_release_pins();
	i2c_set_clientdata(i2c, NULL);
	return 0;
}


/*
关于 dev 和 drv 的匹配
匹配的方式在不同层级有多种方式，其中包括：
struct of_device_id
struct i2c_device_id

目前，仅使用 struct i2c_device_id 好像也是可以匹配的，而且 dt 中的描述也仅仅与 compatible = "yq,tw2823" 有关
也就是说，struct i2c_device_id 中仅仅说明 .name="tw2823"，与可以与 dt 中的 compatible = "yq,tw2823" 匹配，这点有点
不太理解，还需要进一步分析
*/
/*
#define I2C_NAME_SIZE	20

struct i2c_device_id {
	char name[I2C_NAME_SIZE];
	kernel_ulong_t driver_data;	// Data private to the driver
};
在i2c driver驱动中通过以下代码将 device 和 driver 进行匹配
*/
static struct i2c_device_id tw2837_id[] = {
	{.name="tw2837", 0},
	{},
};

/*
用于匹配设备树节点和当前驱动
struct of_device_id {
	char	name[32];
	char	type[32];
	char	compatible[128];
	const void *data;
};
设备树中，通过 .compatible = "xx,xxxx" 匹配
注：如果 dts 中没有描述该结点（具体来说，没有描述 compatible 属性，就不会启动匹配过程，也就不会调用 probe 函数）		
*/
// static const struct of_device_id tw2837_dts_ids[] = {
// 	{.compatible = "yq,tw2823",}, 
// 	{}
// };

/*
MODULE_DEVICE_TABLE（设备类型，设备表）
将 xx_driver_ids 结构输出到用户空间，加载模块时，匹配硬件设备。
（ 设备表的最后一项必须是空）,一般用在热插拔的设备驱动中。
*/
//MODULE_DEVICE_TABLE(of, tw2837_dts_ids);


static unsigned short tw2837_addrs[] = { 0x52, I2C_CLIENT_END };

static struct i2c_driver tw2837_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = DEVICE_NAME,
		//.of_match_table = tw2837_dts_ids,
	},
	.probe          = tw2837_probe,
	.remove         = tw2837_remove,
	.id_table       = tw2837_id,
	.address_list   = tw2837_addrs,
};

// 2. 在插入过程中，通过 init 函数，启动注册设备驱动
static int __init tw2837_init( void )
{
	int ret=0;
    dev_t    devNumber;

    printk(KERN_DEBUG "[TW2837] tw2837_init\n");

    // 普通的 char 设备初始化过程
    if ( alloc_chrdev_region(&devNumber,0,1,DEVICE_NAME) < 0 ){
        printk(KERN_DEBUG "[E] TW2823: alloc_chrdev_region...fault\n");
    } else {
        // 由设备号，获取“主设备号”及“次设备号” dev_t devNumber = MKDEV(dev_major,dev_minor);
 	    dev_major = MAJOR(devNumber);
 	    dev_minor = MINOR(devNumber);
 	    printk(KERN_DEBUG "%d,(%d, %d)\n",devNumber,dev_major,dev_minor);
    }

    tw2837_dev = kmalloc(sizeof(struct tw2837_device_info),GFP_KERNEL);
    if ( !tw2837_dev ){
        goto err1;
    }
    tw2837_dev->dev_cdev = cdev_alloc();    
    
    cdev_init(tw2837_dev->dev_cdev,&tw2837_fops);
    tw2837_dev->dev_cdev->owner = THIS_MODULE;

    if ( cdev_add(tw2837_dev->dev_cdev,devNumber,1) < 0 ){
        goto err2;
    }

    dev_class = class_create( THIS_MODULE,DEVICE_NAME );
    device_create(dev_class,NULL,devNumber,NULL,DEVICE_NAME);
    // 完成 char 设备初始化

    // i2c 设备初始化
    ret = i2c_add_driver(&tw2837_driver);
	if (ret < 0) {
		printk(KERN_WARNING DEVICE_NAME "[E] TW2823:can't add i2c driver: %d\n", ret);
	}
    // 完成 i2c 设备初始化

	return 0;
err1:
    return -1;

err2:
    cdev_del(tw2837_dev->dev_cdev);
    return -2;

}

static void __exit tw2837_exit( void )
{
    
    dev_t devNumber = 0;
	devNumber = MKDEV(dev_major,dev_minor);

	v4l2_int_device_unregister(&tw2837_int_device);


	// 调用该函数，会执行模块的 remove 函数
	i2c_del_driver(&tw2837_driver);

	printk(KERN_DEBUG "[TW2837] tw2837_exit...\n");

	// 销毁设备及设备类，否则再次加载模块时，会由于重复的设备类而错误
	device_destroy(dev_class,devNumber);
    class_destroy(dev_class);
	// 取消设备模块的注册
	unregister_chrdev_region(devNumber, 1);
}
// 结束 模块入口和出口
///////////////////////////////


// module_i2c_driver(tw2837_driver);rm

///////////////////////////////
// 声明模块
module_init(tw2837_init);
module_exit(tw2837_exit);

//MODULE_AUTHOR("Leo Yang")
MODULE_DESCRIPTION("Multi-video tw2823 Driver");
MODULE_LICENSE("GPL");
// 结束 声明模块
///////////////////////////////


// static int tw2837_read_base_setting(void)
// {
// 	int i   = 0;

// 	cmd_t cmdSequence[] = {
// 		// basic config for all channel

// 		// 0x01A4 配置必须为主模式，否则不能输出
// 		{.regAddr=0x01A4,.clrCMD=0xFF,.setCMD=0x80,.ret=0},	// ENC_MODE = MASTER, Must!!
// 		// clock delay output of CLK54MEM pin
// 		{.regAddr=0x017E,.clrCMD=0x0F,.setCMD=0x88,.ret=0},	// MCLKDEL=8
// 		// select video input for channel
// 		{.regAddr=0x0080,.clrCMD=0xFF,.setCMD=0x00,.ret=0},	// CH0=VIN0
// 		{.regAddr=0x0090,.clrCMD=0xCF,.setCMD=0x40,.ret=0}, // CH1=VIN1
// 		{.regAddr=0x00A0,.clrCMD=0xCF,.setCMD=0x80,.ret=0}, // CH2=VIN2
// 		{.regAddr=0x00B0,.clrCMD=0xCF,.setCMD=0xC0,.ret=0},	// CH3=VIN3 
		
// 		// disable all channel
// 		{.regAddr=0x0110,.clrCMD=0xFF,.setCMD=0x00,.ret=0}, // CH0 TURN OFF
// 		{.regAddr=0x0118,.clrCMD=0xFE,.setCMD=0x01,.ret=0}, // CH1 TURN OFF
// 		{.regAddr=0x0120,.clrCMD=0xFD,.setCMD=0x02,.ret=0}, // CH2 TURN OFF
// 		{.regAddr=0x0128,.clrCMD=0xFC,.setCMD=0x03,.ret=0}, // CH3 TURN OFF

// 		// add config,after found the error of video
// 		// turn on video ADC
// 		{.regAddr=0x004C,.clrCMD=0xFF,.setCMD=0x30,.ret=0},
// 		// turn off video output DAC
// 		{.regAddr=0x01A1,.clrCMD=0xFF,.setCMD=0x88,.ret=0}, // dac_pd_cx = dac_pd_yx = 1 ,power down
// 		// turn on DAOYY DAC
// 		{.regAddr=0x01A2,.clrCMD=0xFF,.setCMD=0x90,.ret=0},

// 		// 以下是保留寄存器，DS 没有描述其具体内容，仅说明必须配置为如下内容
// 		// Reserved,normal operation,shuld be set 0x11
// 		{.regAddr=0x000E,.clrCMD=0xFF,.setCMD=0x11,.ret=0},
// 		{.regAddr=0x001E,.clrCMD=0xFF,.setCMD=0x11,.ret=0},
// 		{.regAddr=0x002E,.clrCMD=0xFF,.setCMD=0x11,.ret=0},
// 		{.regAddr=0x003E,.clrCMD=0xFF,.setCMD=0x11,.ret=0},

// 		// Reserved,normal operation,shuld be set
// 		{.regAddr=0x004E,.clrCMD=0xFF,.setCMD=0x05,.ret=0},
// 		{.regAddr=0x004F,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0050,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0051,.clrCMD=0xFF,.setCMD=0x80,.ret=0},
// 		{.regAddr=0x0052,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
// 		{.regAddr=0x0053,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0054,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

// 		// Reserved for test mode,normal operation,shuld be set
// 		{.regAddr=0x0060,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0061,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0062,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0063,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0064,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0065,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0066,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0067,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0068,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0069,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x006A,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x006B,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x006C,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x006D,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x006E,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x006F,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0070,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0071,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0072,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0073,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0074,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

// 		// Reserved,normal operation,shuld be set 
// 		{.regAddr=0x00C7,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},
// 		{.regAddr=0x00C9,.clrCMD=0xFF,.setCMD=0x3C,.ret=0},

// 		// Reserved,normal operation,shuld be set 0x00
// 		{.regAddr=0x0117,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x011F,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0127,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x012F,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

// 		{.regAddr=0x0153,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x017D,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

// 		{.regAddr=0x01B7,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01B8,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01B9,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01BA,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01BB,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01BC,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01BD,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01BE,.clrCMD=0xFF,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x01BF,.clrCMD=0xFF,.setCMD=0x00,.ret=0},

// 		// 以下配置 SDRAM，默认内容为0,属于不可用的数值范围，必须经过设置
// 		// 这个比较奇怪，默认值居然是不能用的 !!!!,至少有个默认值可用啊
// 		// Save addr of SDRAM,4~11 for 64M SDRAM
// 		{.regAddr=0x0102,.clrCMD=0xFF,.setCMD=0x08,.ret=0},
// 		// Recall address for main channel
// 		{.regAddr=0x0112,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
// 		{.regAddr=0x011A,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
// 		{.regAddr=0x0122,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
// 		{.regAddr=0x012A,.clrCMD=0xFF,.setCMD=0x06,.ret=0},
// 		// end add

// 		{.regAddr=0x00C3,.clrCMD=0xFF,.setCMD=0x08,.ret=0},
// 		{.regAddr=0x00C4,.clrCMD=0xFF,.setCMD=0x0F,.ret=0},

// 		// CH0
// 		{.regAddr=0x0002,.clrCMD=0xFF,.setCMD=0x20,.ret=0}, // HDELAY
// 		{.regAddr=0x0006,.clrCMD=0x03,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0003,.clrCMD=0xFF,.setCMD=0xD0,.ret=0}, // HACTIVE
// 		{.regAddr=0x0006,.clrCMD=0x0C,.setCMD=0x08,.ret=0},
// 		{.regAddr=0x0004,.clrCMD=0xFF,.setCMD=0x06,.ret=0}, // VDELAY
// 		{.regAddr=0x0006,.clrCMD=0x10,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0005,.clrCMD=0xFF,.setCMD=0x20,.ret=0}, // VACTIVE
// 		{.regAddr=0x0006,.clrCMD=0x00,.setCMD=0x20,.ret=0},

// 		{.regAddr=0x0081,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [15:8]
// 		{.regAddr=0x0082,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [7:0]
// 		{.regAddr=0x0083,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [15:8]
// 		{.regAddr=0x0084,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [7:0]

// 		{.regAddr=0x0130,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picHL
// 		{.regAddr=0x0131,.clrCMD=0xFF,.setCMD=0xB4,.ret=0},// picHR
// 		{.regAddr=0x0132,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picVT
// 		{.regAddr=0x0133,.clrCMD=0xFF,.setCMD=0x78,.ret=0},// picVB
// 		// CH1
// 		{.regAddr=0x0012,.clrCMD=0xFF,.setCMD=0x20,.ret=0},	// HDELAY
// 		{.regAddr=0x0016,.clrCMD=0x03,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0013,.clrCMD=0xFF,.setCMD=0xD0,.ret=0},	// HACTIVE
// 		{.regAddr=0x0016,.clrCMD=0x0C,.setCMD=0x08,.ret=0},
// 		{.regAddr=0x0014,.clrCMD=0xFF,.setCMD=0x06,.ret=0},	// VDELAY
// 		{.regAddr=0x0016,.clrCMD=0x10,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0015,.clrCMD=0xFF,.setCMD=0x20,.ret=0},	// VACTIVE
// 		{.regAddr=0x0016,.clrCMD=0x00,.setCMD=0x20,.ret=0},

// 		{.regAddr=0x0091,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [15:8]
// 		{.regAddr=0x0092,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [7:0]
// 		{.regAddr=0x0093,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [15:8]
// 		{.regAddr=0x0094,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [7:0]
// 		{.regAddr=0x0134,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picHL
// 		{.regAddr=0x0135,.clrCMD=0xFF,.setCMD=0xB4,.ret=0},// picHR
// 		{.regAddr=0x0136,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picVT
// 		{.regAddr=0x0137,.clrCMD=0xFF,.setCMD=0x78,.ret=0},// picVB
// 		// CH2
// 		{.regAddr=0x0022,.clrCMD=0xFF,.setCMD=0x20,.ret=0},	// HDELAY
// 		{.regAddr=0x0026,.clrCMD=0x03,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0023,.clrCMD=0xFF,.setCMD=0xD0,.ret=0},	// HACTIVE
// 		{.regAddr=0x0026,.clrCMD=0x0C,.setCMD=0x08,.ret=0},
// 		{.regAddr=0x0024,.clrCMD=0xFF,.setCMD=0x06,.ret=0},	// VDELAY
// 		{.regAddr=0x0026,.clrCMD=0x10,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0025,.clrCMD=0xFF,.setCMD=0x20,.ret=0},	// VACTIVE
// 		{.regAddr=0x0026,.clrCMD=0x00,.setCMD=0x20,.ret=0},

// 		{.regAddr=0x00A1,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [15:8]
// 		{.regAddr=0x00A2,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [7:0]
// 		{.regAddr=0x00A3,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [15:8]
// 		{.regAddr=0x00A4,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [7:0]
// 		{.regAddr=0x0138,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picHL
// 		{.regAddr=0x0139,.clrCMD=0xFF,.setCMD=0xB4,.ret=0},// picHR
// 		{.regAddr=0x013A,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picVT
// 		{.regAddr=0x013B,.clrCMD=0xFF,.setCMD=0x78,.ret=0},// picVB
// 		// CH3
// 		{.regAddr=0x0032,.clrCMD=0xFF,.setCMD=0x20,.ret=0},	// HDELAY
// 		{.regAddr=0x0036,.clrCMD=0x03,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0033,.clrCMD=0xFF,.setCMD=0xD0,.ret=0},	// HACTIVE
// 		{.regAddr=0x0036,.clrCMD=0x0C,.setCMD=0x08,.ret=0},
// 		{.regAddr=0x0034,.clrCMD=0xFF,.setCMD=0x06,.ret=0},	// VDELAY
// 		{.regAddr=0x0036,.clrCMD=0x10,.setCMD=0x00,.ret=0},
// 		{.regAddr=0x0035,.clrCMD=0xFF,.setCMD=0x20,.ret=0},	// VACTIVE
// 		{.regAddr=0x0036,.clrCMD=0x00,.setCMD=0x20,.ret=0},

// 		{.regAddr=0x00B1,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [15:8]
// 		{.regAddr=0x00B2,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// vscale [7:0]
// 		{.regAddr=0x00B3,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [15:8]
// 		{.regAddr=0x00B4,.clrCMD=0xFF,.setCMD=0xFF,.ret=0},// hscale [7:0]
// 		{.regAddr=0x013C,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picHL
// 		{.regAddr=0x013D,.clrCMD=0xFF,.setCMD=0xB4,.ret=0},// picHR
// 		{.regAddr=0x013E,.clrCMD=0xFF,.setCMD=0x00,.ret=0},// picVT
// 		{.regAddr=0x013F,.clrCMD=0xFF,.setCMD=0x78,.ret=0},// picVB
// 		// reset all channel ,but control regs
// 		{.regAddr=0x000D,.clrCMD=0xFF,.setCMD=0x04,.ret=0},
// 		{.regAddr=0x001D,.clrCMD=0xFF,.setCMD=0x04,.ret=0},
// 		{.regAddr=0x002D,.clrCMD=0xFF,.setCMD=0x04,.ret=0},
// 		{.regAddr=0x003D,.clrCMD=0xFF,.setCMD=0x04,.ret=0},
// 		// init memory
// 		{.regAddr=0x017F,.clrCMD=0xFF,.setCMD=0x80,.ret=0},
// 		// enable all channel
// 		{.regAddr=0x0110,.clrCMD=0xFF,.setCMD=0x80,.ret=0},
// 		{.regAddr=0x0118,.clrCMD=0xFF,.setCMD=0x81,.ret=0},
// 		{.regAddr=0x0120,.clrCMD=0xFF,.setCMD=0x82,.ret=0},
// 		{.regAddr=0x0128,.clrCMD=0xFF,.setCMD=0x83,.ret=0},
// 	};
	
// 	for (i=0;i<ARRAY_SIZE(cmdSequence);i++) {
// 		tw2837_read_cmd(&cmdSequence[i]);
// 	}
// 	return 0;
// }


// 结束模块
