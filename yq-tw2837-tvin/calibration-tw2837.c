#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <strings.h>
#include <limits.h>
#include <dirent.h>
#include <fcntl.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "tw2837_ioctl.h"

static int dev_fd = 0;

#define _VERSION        ("v1.0")
#define _DATE           ("2019-07-11")

static void help(void)
{
    fprintf(stdout,
	"Usage: [--hdelay] val [--hactive] val [--vdelay] val [--vactive] val\n"
	"  --hdelay val		(0~32)\n"
		"  --hactive val	(0~720)\n"
		"  --vdelay val		(0~32)\n"
		"  --vactive val	(0~288)\n"
	"tw2823-calibration %s,%s\n",_VERSION,_DATE);
    exit(1);
}

channel_property_t prop = {0,32,720,5,288};

int main(int argc, char** argv)
{
	int i = 0;
	int temp = 0;
	int flags = 0;
	
    
    
    if ((dev_fd = open( "/dev/tw2837", O_RDWR)) < 0) {
		printf("[E] Failed to open tw2837\n");
		exit(1);
	}

	// argv[0][..] 实际上是命令行中程序名称，不需要解析
    flags = 0;
    // 参数格式如下： -x command
    // 首先取得 x ，第二阶段再读取 command
    while ( 1+flags < argc ) {
    	if ( argv[1+flags][0] == '-' ){
			// 1 + flags : 指向命令
			// 2 + flags : 指向参数
			if ( !strcmp(argv[ 1 + flags ], "--ch") ){
				temp = strtoul(argv[ 2 + flags ],NULL,10);
				if ( temp > 3 )
					temp = 3;
				else if ( temp < 0 )
					temp = 0 ;
                prop.ch = temp;
			} else if ( !strcmp(argv[ 1 + flags ], "--hdelay") ){
				temp = strtoul(argv[ 2 + flags ],NULL,10);
				if ( temp > 32 )
					temp = 32;
				else if ( temp < 1 )
					temp = 1 ;
				prop.h_delay = temp;
			} else if ( !strcmp(argv[ 1 + flags ], "--hactive") ){
				temp = strtoul(argv[ 2 + flags ],NULL,10);
				if ( temp > 740 )
					temp = 740;
				else if ( temp < 1 )
					temp = 1 ;
				prop.h_active = temp;
			}  else if ( !strcmp(argv[ 1 + flags ], "--vdelay") ){
				temp = strtoul(argv[ 2 + flags ],NULL,10);
				if ( temp > 32 )
					temp = 32;
				else if ( temp < 1 )
					temp = 1 ;
				prop.v_delay = temp;
			}  else if ( !strcmp(argv[ 1 + flags ], "--vactive") ){
				temp = strtoul(argv[ 2 + flags ],NULL,10);
				if ( temp > 288 )
					temp = 288;
				else if ( temp < 1 )
					temp = 1 ;
				prop.v_active = temp;
			} else if ( !strcmp(argv[ 1 + flags ], "--help") ){
				help();
			}
    	}
    	flags++;
	}

    printf("[ TW2837 Calibration ] ch=%i,hdelay=%i,hactive=%i,vdelay=%i,vactive=%i\n",prop.ch,prop.h_delay,prop.h_active,prop.v_delay,prop.v_active);
    ioctl(dev_fd,TW2823_IOC_CH_PROPERTY,prop);
    close(dev_fd);

	return 0;
}