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
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/rtc.h>         /* get the user-level API */
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

#include <linux/fcntl.h>
#include <linux/spinlock.h>
//#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/gpio.h>

#define PM_PWR_BTN                     17
#define GPIO_USB_COPY_BTN         112
#define GPIO_RECOVERY_BTN         113

#define LED_OFF         0x0
#define LED_ON          0x1
#define LED_BLINK1      0x2
#define LED_BLINK2      0x3
#define LED_ENABLE      0x4
#define LED_DISABLE     0x5

extern int pm51_write_gpio(uint8_t pin, uint8_t val);
extern int pm51_read_gpio(uint8_t pin, uint8_t *val);
//IEC-ADD by EDEN for kernel wdtSetWatchdog start at 20151113
#include <linux/jiffies.h>
#include <linux/times.h>
struct wdt_setting {
	int pid;
	int time_set;
	struct timer_list	timer;
	struct list_head	list;
};
static LIST_HEAD(wdt_handlers);
extern int pm51_wdtset(uint8_t val); 
static void  wdt_timeout(unsigned long x);
static int wdt_disable=0;
module_param(wdt_disable, int, S_IRUGO | S_IWUSR);
//IEC-ADD by EDEN for kernel wdtSetWatchdog end at 20151113
extern int pm51_write_led(uint8_t num, uint8_t status);
extern int pm51_read_led(uint8_t num, uint8_t *status);
//extern int pm51_read_version(char* version);

#ifdef DEBUG
# define _DBG(x, fmt, args...) do{ if (x>=DEBUG) printk("%s: " fmt "\n", __FUNCTION__, ##args); } while(0);
#else
# define _DBG(x, fmt, args...) do { } while(0);
#endif

int rst_btn_count = 0;
u8 copy_btn_state=1;
u8 pwr_btn_state=1;
u8 rst_btn_state=1;

void iec_disk_access(int index, int act);
		
// SATA LED control array 
static u32 access_led[2]={LED_OFF};
static u8 qc_cur[2];          // record current qc state
static atomic_t qc_new[2];    // record qc_cmd coming
static u8 qc_fail[2]={-1,-1};          // record qc fail led

// Note: Be careful the disk index since we don' have DOM on evansport
//	   platoform. Therefore the tray_id is in range 1~2 but port id is 0~1.
void iec_disk_access(int index, int act)
{
	_DBG(1, "index %d: %d\n", index, act);

	// Map tray_id to port index;
	if (act >= LED_ENABLE)
		index--;

	//only handle 2 disk
	if ( (index < 0)||(index > 1) )
		return;

	// set enable or disable by tray_id
	switch(act){
	case LED_ENABLE:	
		atomic_set(&qc_new[index], LED_ON);
		qc_fail[index]=0;
		access_led[index] = LED_ON;
		goto out;
		break;
	case LED_DISABLE:
		qc_fail[index]=1;
		access_led[index] = LED_OFF;
		goto out;
		break;
	}

	// if LED is disabled, do nothing
	if (access_led[index] == LED_OFF)
		goto out;

	switch(act){
	case LED_ON:
		/*
		 * qc free; 
		 * since we use sata_led_routine() for qc coming blinking,
		 * do nothing here.
		 */
		break;
	case LED_OFF:
		// qc comes; set blink randow
		atomic_set(&qc_new[index], 7);
		break;
	}

out:
	return;
}
EXPORT_SYMBOL(iec_disk_access);

static ssize_t proc_board_io_write(struct file *file,
                    const char __user * buf, size_t length, loff_t * ppos)
{
	char *buffer, buf1[20], item[20];
	int i, err, num, val1, val2;
	u8 status = 0;

	if (!buf || length > PAGE_SIZE)
		return -EINVAL;

	err = -ENOMEM;
	buffer = (char *) __get_free_page(GFP_KERNEL);
	if (!buffer)
		goto out2;

	err = -EFAULT;
	if (copy_from_user(buffer, buf, length))
		goto out;

	err = -EINVAL;
	if (length < PAGE_SIZE) {
		buffer[length] = '\0';
#define LF	0xA
		if (length > 0 && buffer[length - 1] == LF)
			buffer[length - 1] = '\0';
	} else if (buffer[PAGE_SIZE - 1])
		goto out;

	memset(buf1, 0, sizeof(buf1));

    /*
     * Usage: echo "<item> [<index>] <value>" > /proc/BOARD_io
     *        value    0:Off; 1:On; 2:Blink;
     *        "S_LED 1-N 0|1|2"               * LED SATA 1-N ERROR led
     *        "A_LED 1-N 0|1|2"               * LED SATA 1-N ACTIVE led
     *        "U_LED 0|1|2"                   * USB BUSY led
     *        "UF_LED 0|1|2"                  * USB ERROR led
     *        "PWR_LED 0|1|2"                 * LED System Status
     */

	i = sscanf(buffer, "%s %d %d\n", item, &val1, &val2);
	if (i == 3){
		num = val1;
		status = val2;
	} else if(i == 2){
		status = val1;
	}

	if (i > 1)
		_DBG(3, "input %s %d\n", item, status);

	if (!strncmp(item, "S_LED", strlen("S_LED"))){
		if (i != 3) goto parse_done;
		if (num > 0) {			
			pm51_write_led(num+2, status);
		}
	} else if (!strncmp(item, "A_LED", strlen("A_LED"))){
		if (i != 3) goto parse_done;
		if (num > 0) {
			pm51_write_led(num, status);
		}
	} else if (!strncmp(item, "U_LED", strlen("U_LED"))){
		if (i != 2) goto parse_done;
		pm51_write_led(7, status);
	} else if (!strncmp(item, "UF_LED", strlen("UF_LED"))){
		if (i != 2) goto parse_done;
	} else if (!strncmp(item, "W_LED", strlen("W_LED"))){
		if (i != 2) goto parse_done;
		pm51_write_led(5, status);
	} else if (!strncmp(item, "WF_LED", strlen("WF_LED"))){
		if (i != 2) goto parse_done;
		pm51_write_led(6, status);
	} else if (!strncmp(item, "PWR_LED", strlen("PWR_LED"))){
		if (i != 2) goto parse_done;
		pm51_write_led(0, status);
	} else if (!strncmp(item, "ZB_RST", strlen("ZB_RST"))){
		if (i != 2) goto parse_done;
		pm51_write_gpio(5, status);
	} else if (!strncmp(item, "W_DIS", strlen("W_DIS"))){
		if (i != 2) goto parse_done;
		pm51_write_gpio(21, status);
	} else if (!strncmp(item, "WOL_EN", strlen("WOL_EN"))){
		if (i != 2) goto parse_done;
		pm51_write_gpio(4, status);
	}		

parse_done:

	err = length;
out:
	free_page((unsigned long) buffer);
out2:
	*ppos = 0;

	return err;
}


static int proc_board_io_show(struct seq_file *m, void *v)
{
	u8 val = 0;
	char LED_STATUS[4][8];

	sprintf(LED_STATUS[LED_ON], "ON");
	sprintf(LED_STATUS[LED_OFF], "OFF");
	sprintf(LED_STATUS[LED_BLINK1], "BLINK");
	sprintf(LED_STATUS[LED_BLINK2], "-");

	seq_printf(m, "MODELNAME: Barcelona\n");
	seq_printf(m, "MBTYPE: Barcelona\n");
	
	seq_printf(m,"Recovery button: %s\n", rst_btn_state?"OFF":"ON");
	seq_printf(m,"Copy button: %s\n", copy_btn_state?"OFF":"ON");

	pm51_read_led(7,&val);
	switch(val){
	case 1:
		seq_printf(m, "U_LED: ON\n");
		seq_printf(m, "UF_LED: OFF\n");
		break;
	case 2:
		seq_printf(m, "U_LED: BLINK\n");
		seq_printf(m, "UF_LED: OFF\n");
		break;
	case 3:
		seq_printf(m, "U_LED: BLINK1\n");
		seq_printf(m, "UF_LED: OFF\n");
		break;
	case 4:
		seq_printf(m, "U_LED: OFF\n");
		seq_printf(m, "UF_LED: ON\n");
		break;
	default:
		seq_printf(m, "U_LED: OFF\n");
		seq_printf(m, "UF_LED: OFF\n");		
		break;
	}		

	pm51_read_led(0,&val);
	if(val>3)val=3;	
	seq_printf(m, "LED_Power: %s\n", LED_STATUS[val]);

	seq_printf(m, "PM_GPIO17: %s\n", pwr_btn_state ? "HIGH" : "LOW");

	return 0;
}

static int proc_board_io_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_board_io_show, NULL);
}

static struct file_operations proc_board_io_operations = {
	.open = proc_board_io_open,
	.read = seq_read,
	.write = proc_board_io_write,
	.llseek = seq_lseek,
	.release = single_release,
};

// ----------------------------------------------------------
#define MESSAGE_LENGTH 80
#define MY_WORK_QUEUE_NAME "board_wq"    // length must < 10
#define WORK_QUEUE_TIMER_1 250
#define WORK_QUEUE_TIMER_2 50
static DECLARE_WAIT_QUEUE_HEAD(board_event_queue);
static char Message[MESSAGE_LENGTH];
static int module_die = 0;    /* set this to 1 for shutdown */
static u32 dyn_work_queue_timer = WORK_QUEUE_TIMER_2;
static struct workqueue_struct *my_workqueue;
static void intrpt_routine(struct work_struct *unused);
static void sata_led_routine(struct work_struct *ws);
static DECLARE_DELAYED_WORK(btn_sched, intrpt_routine);
static DECLARE_WORK(sata_work, sata_led_routine);

static void intrpt_routine(struct work_struct *unused)
{		
	// send event while button is pressed. 'ON' means 'pressed'.
	pm51_read_gpio(PM_PWR_BTN, &pwr_btn_state); //With Watchdog reset ever 50ms
	if (0 == gpio_request(GPIO_USB_COPY_BTN,"usb_copy_btn")) {
		copy_btn_state = gpio_get_value(GPIO_USB_COPY_BTN);
		gpio_free(GPIO_USB_COPY_BTN);
	}
	if (0 == gpio_request(GPIO_RECOVERY_BTN,"recovery_btn")) {
		rst_btn_state = gpio_get_value(GPIO_RECOVERY_BTN);
		gpio_free(GPIO_RECOVERY_BTN);
	}	
	if(copy_btn_state == 0) {
		rst_btn_count=0;
		sprintf(Message, "Copy ON\n");
		_DBG(2, "Copy ON\n");
		wake_up_interruptible(&board_event_queue);
	} else {	
		if (pwr_btn_state == 0) {
			rst_btn_count=0;
			sprintf(Message, "PWR ON\n");
			wake_up_interruptible(&board_event_queue);
		} else {		
			if(rst_btn_state == 0) {
				rst_btn_count=0;							
				sprintf(Message, "RST2DF ON\n");
				_DBG(2, "RST2DF ON\n");
				wake_up_interruptible(&board_event_queue);
			} else {
				if (rst_btn_count<5) {
					rst_btn_count=rst_btn_count+1;
					sprintf(Message, "NOBTN ON\n");
					_DBG(2, "NOBTN ON\n");
					wake_up_interruptible(&board_event_queue);
				}
			}			
		}
	}	
	// keep intrp_routine queueing itself
	if (module_die == 0)
		queue_delayed_work(my_workqueue, &btn_sched, dyn_work_queue_timer);
	if (wdt_disable>1)pm51_wdtset(1); //IEC-ADD by EDEN for wdt timeout reboot system at 20151113
}

/*
 * This routine is for pm51/pca9532 which are very low-speed devices to
 * control LED blink action when there is qc coming instead of pulling LED
 * on/off in disk_access() immediately.
 */
static void sata_led_routine(struct work_struct *ws)
{
	int i;
	u8 act;
	u8 max_disk = 0, sata_led_on = 1, sata_led_off = 0;

	max_disk = 2;
	sata_led_on = LED_ON;
	sata_led_off = LED_OFF;

	// only handle model MAX disk amount
	for (i = 0; i < max_disk; i++){
		switch(access_led[i]){
		case LED_ON:
			act = atomic_read(&qc_new[i]);
			atomic_set(&qc_new[i], sata_led_on);
			break;
		default:
			act = sata_led_off;
			break;
		}

		// only set LED while status has changed
		if(qc_cur[i] != act){	
/*		
LED_NUM		
0 : Power LED
1 : Sata1 LED
2 : Sata2LED
3 : WLAN
4 : USB
LED_STATUS
0 : LED off
1 : LED on
2 : LED Blink Slow
3 : LED Blink fast			
4 : LED fail	
5 : LED fail Blank //not use now
*/
			pm51_write_led(i+1, act);
			if (qc_fail[i]>=0) {
				pm51_write_led(i+3, qc_fail[i]);
				qc_fail[i]=-1;
			}		
			qc_cur[i] = act;
		}
	}

	msleep(50);
	// keep work routine queueing itself
	if (!module_die)
		queue_work(my_workqueue, &sata_work);
}

// ----------------------------------------------------------
static ssize_t board_event_read(struct file *file, char __user * buffer,
                                 size_t length, loff_t * ppos)
{
	static int finished = 0;
	int i;
	if (finished) {
		finished = 0;
		return 0;
	}
//      printk(KERN_DEBUG "process %i (%s) going to sleep\n",
//           current->pid, current->comm);
//	interruptible_sleep_on(&board_event_queue);
	wait_event_interruptible(board_event_queue, strlen(Message) > 0);
//      printk(KERN_DEBUG "awoken %i (%s)\n", current->pid, current->comm);
	for (i = 0; i < length && Message[i]; i++)
		put_user(Message[i], buffer + i);

	finished = 1;
	return i;
}

static struct file_operations proc_board_event_operations = {
	.read = board_event_read,
};

static void wdt_timeout(unsigned long x)
{
	struct wdt_setting *wdtset = (struct wdt_setting *)x;
	printk(KERN_ERR "wdt_timeout pid=%d, time_set=%d\n", wdtset->pid,wdtset->time_set);
	if(wdt_disable==0)wdt_disable=2;
}

static ssize_t proc_wdtsetting_write(struct file *file,
                    const char __user * buf, size_t length, loff_t * ppos)
{
	struct wdt_setting *wdtset;
	char *buffer, buf1[20];
	int i, err, pid;
	uint32_t time_set;

	if (!buf || length > PAGE_SIZE)
		return -EINVAL;

	err = -ENOMEM;
	buffer = (char *) __get_free_page(GFP_KERNEL);
	if (!buffer)
		goto out2;

	err = -EFAULT;
	if (copy_from_user(buffer, buf, length))
		goto out1;

	err = -EINVAL;
	if (length < PAGE_SIZE) {
		buffer[length] = '\0';
#define LF	0xA
		if (length > 0 && buffer[length - 1] == LF)
			buffer[length - 1] = '\0';
	} else if (buffer[PAGE_SIZE - 1])
		goto out1;

	memset(buf1, 0, sizeof(buf1));

    /*
     * Usage: echo "<item> <value>" > /proc/WdtSetting
     *        value    0~255; for task dog disable ~ 255 sec timeout
     */
     
	i = sscanf(buffer, "%d %d\n", &pid, &time_set);
	
	if (i < 2) {
		err = -EINVAL;
		goto out1;
	}

	list_for_each_entry(wdtset, &wdt_handlers, list) {
		if (wdtset->pid == pid) {
			del_timer_sync(&wdtset->timer);
			if (time_set==0) {
				list_del(&wdtset->list);
				kfree(wdtset);
				goto out;
			} 
			goto reset_timedelay;
		}
	}
	if (time_set==0)goto out;
	wdtset = kzalloc(sizeof(struct wdt_setting), GFP_KERNEL);
	if (wdtset == NULL) {		
		err = -ENOMEM;
		goto out1;
	}
	wdtset->pid=pid;
	list_add(&wdtset->list,&wdt_handlers);
reset_timedelay:
	wdtset->time_set=time_set;
	setup_timer(&wdtset->timer, wdt_timeout, (unsigned long)wdtset);
	wdtset->timer.expires = jiffies + time_set*HZ;
	add_timer(&wdtset->timer);
out:	
	err = length;
out1:
	free_page((unsigned long) buffer);
out2:
	*ppos = 0;
	
	return err;
}

static struct file_operations proc_wdtsetting_operations = {
	.write = proc_wdtsetting_write,
};

static int sys_notify_reboot(struct notifier_block *nb, unsigned long event,
                                void *p)
{
//IEC-Mask by EDEN for keep boot mode led start at 20151113
#if 0
		// set LEDs
		pm51_write_led(0, 2); 
#endif
//IEC-Mask by EDEN for keep boot mode led end at 20151113
		pm51_write_led(1, 0);
		pm51_write_led(2, 0);
		pm51_write_led(3, 0);
		pm51_write_led(4, 0);
		pm51_write_led(5, 0);
		pm51_write_led(6, 0);
		pm51_write_led(7, 0);

	return NOTIFY_DONE;
	
}



static struct notifier_block sys_notifier_reboot = {
	.notifier_call = sys_notify_reboot,
	.next = NULL,
	.priority = 0
};

static int pm_notify_sleep(struct notifier_block *nb, unsigned long event,
                                void *p)
{
	#define PM_SUSPEND_PREPARE	0x0003 /* Going to suspend the system */
	#define PM_POST_SUSPEND		0x0004 /* Suspend finished */
	
	if ( event == PM_SUSPEND_PREPARE) {
		wdt_disable=1;
		pm51_wdtset(0); //IEC-ADD by EDEN for kernel disable wdtSetWatchdog at 20151113
		printk(KERN_INFO "PM_SUSPEND_PREPARE : Disable WDT\n");
	} else if ( event == PM_POST_SUSPEND) {
		printk(KERN_INFO "PM_POST_SUSPEND : Enable WDT\n");
		pm51_wdtset(100); //IEC-ADD by EDEN for kernel init wdtSetWatchdog 10 Sec at 20151113
		wdt_disable=0;	
	}
	return NOTIFY_DONE;	
}

static struct notifier_block pm_notifier_board = {
	.notifier_call = pm_notify_sleep,
	.next = NULL,
	.priority = 0
};

static __init int board_io_init(void)
{
	int ret;
	struct proc_dir_entry *pde;
	u32 n = 0;
	// unsigned int id;
	// TODO: initialize USB power enable
	/*
	 * struct pci_dev *pdev = pci_get_device(PCI_VENDOR_ID_INTEL, PCI_INTELCE_GPIO_DEVICE_ID, NULL);
	 * struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	 */
	// intelce_get_soc_info(&id, NULL);
	// if (id != CE5300_SOC_DEVICE_ID) 
	//	return -ENODEV;
	// set LEDs
//IEC-Mask by EDEN for keep boot mode led start at 20151113
#if 0
	pm51_write_led(0, 2);
#endif 
//IEC-Mask by EDEN for keep boot mode led end at 20151113
	pm51_write_led(1, 0);
	pm51_write_led(2, 0);
	pm51_write_led(3, 0);
	pm51_write_led(4, 0);
	pm51_write_led(5, 0);
	pm51_write_led(6, 0);
	pm51_write_led(7, 0);
	// get current SATA LED
	for (n = 0; n < 2; n++)
		qc_cur[n]=LED_OFF;

#if 0
-       entry = create_proc_entry("sequence", 0, NULL);
-       if (entry)
-               entry->proc_fops = &ct_file_ops;
+       entry = proc_create("sequence", 0, NULL, &ct_file_ops);
#endif
	// create BOARD_io and theucs_event proc nodes
	// pde = create_proc_entry("BOARD_io", 0, NULL);
	pde = proc_create("BOARD_io", 0, NULL, &proc_board_io_operations);	
	if (!pde) {
		printk(KERN_ERR "board_io: cannot create /proc/BOARD_io.\n");
		ret = -ENOENT;
		goto io_out;
	}
	// pde->proc_fops = &proc_board_io_operations;
	// pde = create_proc_entry("BOARD_event", S_IRUSR, NULL);
	pde = proc_create("BOARD_event", S_IRUSR, NULL, &proc_board_event_operations);
	if (!pde) {
		printk(KERN_ERR "board_io: cannot create /proc/BOARD_event.\n");
		ret = -ENOENT;
		goto event_out;
	}
	// pde->proc_fops = &proc_board_event_operations;
	// add our work queue
	my_workqueue = create_workqueue(MY_WORK_QUEUE_NAME);
	if (my_workqueue) {
		queue_delayed_work(my_workqueue, &btn_sched, dyn_work_queue_timer);
		// need a routine thread for SATA ACT_LED
		queue_work(my_workqueue, &sata_work);
			
		init_waitqueue_head(&board_event_queue);
	} else {
		printk(KERN_ERR "board_io: error in board_io_init\n");
		ret = -ENOENT;
		goto wq_out;
	}
//IEC-ADD by EDEN for userspace init wdtSetWatchdog start at 20151113
	pde = proc_create("WdtSetting", S_IRUSR, NULL, &proc_wdtsetting_operations);
	// pde = create_proc_entry("WdtSetting", S_IRUSR, NULL);
	if (!pde) {
		printk(KERN_ERR "board_io: cannot create /proc/WdtSetting.\n");
		ret = -ENOENT;
		goto wdt_out;
	}
	// pde->proc_fops = &proc_wdtsetting_operations;
//IEC-ADD by EDEN for userspace init wdtSetWatchdog end at 20151113

	register_pm_notifier(&pm_notifier_board);

	pm51_wdtset(100); //IEC-ADD by EDEN for kernel init wdtSetWatchdog 10 Sec at 20151113


	register_reboot_notifier(&sys_notifier_reboot);
	return 0;
wdt_out:
	remove_proc_entry("WdtSetting", NULL);
wq_out:
	remove_proc_entry("BOARD_event", NULL);
event_out:
	remove_proc_entry("BOARD_io", NULL);
io_out:
	return ret;
}

static __exit void board_io_exit(void)
{
	pm51_wdtset(0); //IEC-ADD by EDEN for kernel disable wdtSetWatchdog at 20151113
	module_die = 1;                     // If cleanup wants us to die 
	cancel_delayed_work(&btn_sched);    // no "new ones" 
	flush_workqueue(my_workqueue);      // wait till all "old ones" finished 
	destroy_workqueue(my_workqueue);

	remove_proc_entry("WdtSetting", NULL);
	remove_proc_entry("BOARD_event", NULL);
	remove_proc_entry("BOARD_io", NULL);

	unregister_reboot_notifier(&sys_notifier_reboot);
	unregister_pm_notifier(&pm_notifier_board);

}

MODULE_AUTHOR("Eden Laio<liao.eden@inventec.com>");
MODULE_DESCRIPTION(" Inventec Barcelona MB Driver and board depend io operation");
MODULE_LICENSE("GPL");
module_init(board_io_init);
module_exit(board_io_exit);
