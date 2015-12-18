/*
 *  Copyright (C) 2015 Inventec Technology Corp. 
 *
 *      Maintainer: Eden Laio<liao.eden@inventec.com>
 *
 *      Driver for Inventec Barcelona board's I/O
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/rtc.h>		/* get the user-level API */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/vt_kern.h>
#include <linux/reboot.h>
#include <linux/pci.h>

#ifdef DEBUG
# define _DBG(x, fmt, args...) do{ if (x>=DEBUG) printk("%s: " fmt "\n", __FUNCTION__, ##args); } while(0);
#else
# define _DBG(x, fmt, args...) do { } while(0);
#endif

/* polling 8051 FW I/O Module version */
/* Assumption: VUart I/O port is 0x3E8 */
#define VUART_BASE                   0x3e8
#define VUART_LSR                    (VUART_BASE + 5)

// PIC commands
#define CMD_NONE                     0
#define CMD_IR                       1
#define CMD_CEC                      2
#define CMD_ADDRESS                  3
#define CMD_POWER                    4
#define CMD_CEC_ACK                  5
#define CMD_ACK                      6
#define CMD_NAK                      7
#define CMD_START                    8
#define CMD_VERSION                  9
#define CMD_PROGRAM                 10
#define CMD_RS232                   11
#define CMD_IR_REPEAT_START         12
#define CMD_IR_REPEAT_STOP          13
#define CMD_IR_REPEAT_MODE          14
#define CMD_IO_EVENT                15
#define CMD_IO_TIMER_VALUE          16
#define CMD_IR_PLUS                 17
#define CMD_CHECKSUM                18
#define CMD_PWM_PANEL               19
#define CMD_PWM_FAN                 20
#define CMD_CT                      21
#define CMD_GPIO                    22
#define CMD_POWERUPKEYS             23
#define CMD_LTDC                    24
#define CMD_WDT_SET                 25
#define CMD_WDT_HEARTBEAT           26
#define CMD_IR_PLUS_REPEAT_START    27
#define CMD_IR_PLUS_REPEAT_STOP     28
#define CMD_GPIO_READ               29
//#define CMD_PWM_LOGO                29
#define CMD_FACMODE                 30
#define CMD_SNIFFER                 31

// Communications Control bytes
#define SYNC                        0xaa
#define MAX_MSG_LENGTH              255 // max buffer size 
#define MIN_MSG_LENGTH              2   // min message size is 2 chars for
                                        // single-byte ACK/NAK message

#define LR_PIC_GPIO                 22  // PIC GPIOs

#define BAUDRATE_9600               1041

extern u8 pwr_btn_state;
static char version_8051[MAX_MSG_LENGTH];
static DEFINE_MUTEX(uart_lock);

// return valid hex digit or 16 to indicate erroneous input value
static uint8_t charToHex( uint8_t currentChar )
{
	uint8_t val;
	if ( currentChar >= '0' && currentChar <= '9' )
	{
		val = currentChar - '0';
	}
	else if ( currentChar >= 'A' && currentChar <= 'F' )
	{
		val = currentChar - 'A' + 10;
	}
	else if ( currentChar >= 'a' && currentChar <= 'f' )
	{
		val = currentChar - 'a' + 10;
	}
	else
	{
		val = 16;
	}

	return val;
}

static unsigned char byteToChar(unsigned char val)
{
	unsigned char ret;

	if (val < 10)
		ret = '0' + val;
	else
		ret = 'A' + val - 10;

	return ret;
}

static int encodeCMD( unsigned char* inBuffer, int inLength, unsigned char* outputBuffer, int* outputLength )
{
	unsigned char* ptr = outputBuffer;
	char checkSum = 0;
	int length = 2 * inLength + 4;  // total length of output buffer if all goes well
	unsigned char val;
	int i;

	if ( inLength <= 0 )
		return -1;

	if ( inBuffer == NULL || outputBuffer == NULL || outputLength == NULL )
		return -1;

	// now convert data to ASCII and assemble output buffer
	*outputLength = length;

	*ptr++ = SYNC;			  // sync byte
	*ptr++ = 2 * inLength;	  // length of message characters in buffer (excludes checksum chars)
	for( i = 0; i < inLength; i++ )
	{
		val = *(inBuffer + i);
		checkSum = checkSum + val;
		*ptr = byteToChar((val >> 4) & 0x0F);
		ptr++;
		*ptr = byteToChar(val & 0x0F);
		ptr++;
	}

	// put checksum into buffer
	//checkSum = -checkSum & 0xFF;  // don't bother inverting checksum -- just use it as is
	_DBG(1, "output checksum %x\n", checkSum );

	*ptr = byteToChar((checkSum >> 4) & 0x0F);
	ptr++;
	*ptr = byteToChar(checkSum & 0x0F);
	//sprintf( (char*)ptr++, "%0X", (checkSum >> 4) & 0x0F );
	//sprintf( (char*)ptr, "%0X", checkSum & 0x0F );

	_DBG(1, "encodeOutputBuffer: " );
	for( i = 0; i < length; i++ )
		_DBG(1, "%0x ", outputBuffer[i] );
	_DBG(1, "\n\n" );

	return 0;
}

static int pm51_rw(uint8_t sendCmd[], int send_len, uint8_t* recvbuffer)
{
	int i, j, ret;
	uint8_t len;
	uint8_t bh,bl;
	uint8_t lsr;
	uint8_t spin;

	mutex_lock(&uart_lock);
/*	ret = mutex_trylock(&uart_lock);
	if (ret)
		return ret;
*/
	_DBG(2, "start\n");

	// Clear recvbuffer
#if 0	
	for (i=0; i<sizeof(recvbuffer); i++)
		recvbuffer[i] = 0;
#endif	

	// Clear the vuart
	spin = 1;
	do {
		inb(VUART_BASE);
		lsr = inb(VUART_LSR);
	} while((lsr & 0x1) && (spin++ != 0));

	// Enter connected state with 8051 by sending sendCmd

	// Send encoded start command
	for(i=0; i < send_len; i++)
	{
		outb(sendCmd[i], VUART_BASE);
		usleep_range(BAUDRATE_9600, BAUDRATE_9600+100);
	}

	// Decode protocol stream from 8051 FW
	// First reply will be ack then will be the version answer...

	// Wait for data ready
	spin=1;
	do {
		lsr = inb(VUART_LSR);
	} while(((lsr & 0x1) == 0) && (spin++ != 0));

	len = inb(VUART_BASE); // SYNC
	if(len != SYNC)
	{
		printk(KERN_ERR "8051 FW I/O Module does not exist\n");
		ret = -ENODEV;
		goto out;
	}
	if (sendCmd[1]==6)
	{
		ret = 0;
		goto out;
	}

	ret = -EPERM;
	// The first set of bytes is the protocol ACK so lets read bytes until
	// we get the next sync = SYNC
	spin=1;
	do {
	   len = inb(VUART_BASE);
	   usleep_range(BAUDRATE_9600, BAUDRATE_9600+100);
	} while((len != SYNC) && (spin++ != 0));

	// Check for spinout  
	if(len != SYNC)
	{
		printk(KERN_ERR "8051 Target parameter does not exist (1)\n");
		goto out;
	}

	// Now the remaining protocol with encoded version info
	// SYNC (already read above),LEN (of Encoded bytes),
	// Encoded bytes = CMD_VERSION,Len (of version data),Version data
	len = inb(VUART_BASE);
	if (len == 0)
		printk(KERN_ERR "8051 Target parameter does not exist (2)\n");
	else
		_DBG(1, "8051 Target parameter length: %d", len);

	j = 0;
	for (i=0; i < len/2; i++) {
		bh = charToHex(inb(VUART_BASE));
		usleep_range(BAUDRATE_9600, BAUDRATE_9600+100);
		bl = charToHex(inb(VUART_BASE));
		usleep_range(BAUDRATE_9600, BAUDRATE_9600+100);
		if (i==0) {
			ret=(bh << 4) | bl;
		} else if (i==1) {
			if (ret ==CMD_PWM_FAN) {
				recvbuffer[j++] = (bh << 4) | bl;
				_DBG(1, "%c %d", recvbuffer[j-1], recvbuffer[j-1]);
			}
		//First four bytes are the encoded CMD enum and the length which is redundant, which is redundant.
		} else {
			recvbuffer[j++] = (bh << 4) | bl;
			_DBG(1, "%c %d", recvbuffer[j-1], recvbuffer[j-1]);
		}	
	}

	_DBG(1, "\n");

	// Clear the vuart
	spin = 1;
	do {
		inb(VUART_BASE);
		lsr = inb(VUART_LSR);
	} while((lsr & 0x1) && (spin++ != 0));

	ret = 0;
out:
	_DBG(2, "end\n");
	mutex_unlock(&uart_lock);
	return ret;
}

int pm51_wdtset(uint8_t val)
{
	enum wdtActions {
		WDT_WARM_RESET=1,
		WDT_COLD_RESET,
		WDT_POWER_OFF,
		WDT_CAT_POWER_OFF,
		WDT_LAST_ACTION
	};
	int ret = 0;
	uint8_t cmd[13] = {CMD_WDT_SET,WDT_COLD_RESET,val,0,val,0,1,WDT_WARM_RESET,val,0,val,0,1};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_wdtset);

int pm51_write_gpio(uint8_t pin, uint8_t val)
{
	int ret = 0;
	uint8_t cmd[3] = {CMD_GPIO, pin, val};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	_DBG(3, "%d, %d\n", pin, val);

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_write_gpio);

int pm51_read_gpio(uint8_t pin, uint8_t *val)
{
	int ret = 0;
	uint8_t cmd[2] = {CMD_GPIO_READ, pin};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	_DBG(2, "%d\n", pin);

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
	if (ret == 0)
		*val = recvbuffer[0];
	_DBG(2, "value %d\n", *val);

out:
	return ret;
}
EXPORT_SYMBOL(pm51_read_gpio);

int pm51_write_led(uint8_t num, uint8_t staus)
{
	int ret = 0;
	uint8_t cmd[3] = {19, num, staus};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	_DBG(3, "%d, %d\n", num, staus);

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_write_led);

int pm51_read_led(uint8_t num, uint8_t *staus)
{
	int ret = 0;
	uint8_t cmd[2] = {19, num+8};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	_DBG(3, "%d\n", num);

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
	if (ret == 0)
		*staus = recvbuffer[0];
	_DBG(2, "staus %d\n", *staus);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_read_led);

int pm51_write_fan_duty(uint8_t duty)
{
	int ret = 0;
	uint8_t cmd[3] = {CMD_PWM_FAN, duty,0};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	_DBG(3, "%d\n", duty);

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_write_fan_duty);

int pm51_read_fan_tach(uint16_t *rpm)
{
	int ret = 0;
	uint8_t cmd[2] = {CMD_PWM_FAN, 100};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;
	uint8_t recvbuffer[MAX_MSG_LENGTH] = {0};

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}
	ret = pm51_rw(sendCmd, send_len, recvbuffer);
	if (ret == 0)
		*rpm = recvbuffer[1]<<8|recvbuffer[0];
	_DBG(2, "value %d\n", *rpm);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_read_fan_tach);

int pm51_read_version(char* version)
{
	int ret = 0;
	uint8_t cmd[1] = {CMD_VERSION};
	uint8_t sendCmd[MAX_MSG_LENGTH] = {0};
	int send_len = 0;

	_DBG(2, "\n");

	ret = encodeCMD(cmd, sizeof(cmd), sendCmd, &send_len);
	if (ret != 0){
		printk(KERN_ERR "coding buffer problem\n");
		goto out;
	}

	ret = pm51_rw(sendCmd, send_len, (uint8_t*)version);
	_DBG(2, "value %s\n", version);
out:
	return ret;
}
EXPORT_SYMBOL(pm51_read_version);

static ssize_t
fan_io_proc_write(struct file *file, const char __user * buf,
                   size_t length, loff_t * ppos)
{
	unsigned    long val;
	int err;

	if (!buf)
		return -EINVAL;
	if (1 == sscanf(buf, "%lu", &val)) {
		if ((val>100)||(val<0)) return -EINVAL;
		if (val>99)val=99;
		pm51_write_fan_duty((uint8_t)val);		
	} else return -ENOMEM;
	err = length;

	return err;
}

static int fan_io_proc_show(struct seq_file *m, void *v)
{
	int ret = 0;
	uint16_t val;
	ret = pm51_read_fan_tach(&val);
	if (ret)
		seq_printf(m, "read fails!\n");
	else
		seq_printf(m, "%d\n", val);

	return ret;
}

static int fan_io_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fan_io_proc_show, NULL);
}

static struct file_operations proc_fan_io_operations = {
	.open = fan_io_proc_open,
	.read = seq_read,
	.write = fan_io_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

int pm51_init_procfs(void)
{
	struct proc_dir_entry *pde;
#if 0
      	entry = create_proc_entry("sequence", 0, NULL);
        if (entry)
                entry->proc_fops = &ct_file_ops;
        entry = proc_create("sequence", 0, NULL, &ct_file_ops);
#endif

#if 0
	pde = create_proc_entry("FAN_io", 0, NULL);
	if (!pde)
		return -ENOMEM;
	pde->proc_fops = &proc_fan_io_operations;
#endif

	pde = proc_create("FAN_io", 0, NULL, &proc_fan_io_operations);

	if (!pde)
		return -ENOMEM;
	return 0;
}

void pm51_exit_procfs(void)
{
	remove_proc_entry("FAN_io", NULL);
}

static int __init pm51_gpio_init(void)
{
	int err = 0;	
	printk(KERN_INFO "pm51_gpio init\n");

	err = pm51_read_version(version_8051);
	if (err)
		printk(KERN_ERR "Version read fails!\n");
	else
		printk(KERN_INFO "8051 F/W Version: %s\n", version_8051);

	err = pm51_init_procfs();

	return err;
}

static void __exit pm51_gpio_exit(void)
{
	pm51_exit_procfs();
	printk(KERN_INFO "pm51_gpio exit\n");
}

MODULE_AUTHOR("Eden Laio<liao.eden@inventec.com>");
MODULE_DESCRIPTION("PIC UART GPIO Driver");
MODULE_LICENSE("GPL");
module_init(pm51_gpio_init);
module_exit(pm51_gpio_exit);
