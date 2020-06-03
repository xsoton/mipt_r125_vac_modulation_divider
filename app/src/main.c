#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdarg.h>

#include <gpib/ib.h>

// === config ===

#define OSC_NAME "/dev/usbtmc_dso4254c"

#ifdef DEV_AKIP_TMC
	#define PPS_NAME "/dev/usbtmc_1142_3g"
	#define VM_NAME "/dev/usbtmc_v7_78_1"
#else
	#define PPS_NAME "AKIP-1142/3G"
	#define VM_NAME "AKIP-V7-78/1"
#endif

// === time ===

#define STEP_DELAY 1.0e6 // us

// === pps ===

#define VOLTAGE_MIN 0.0
#define VOLTAGE_MAX 10.0
#define VOLTAGE_STEP 0.1
#define CURRENT_MAX 0.1
#define LASER_VOLTAGE 5.0
#define LASER_CURRENT 0.15

// === threads ===

void *commander(void *);
void *worker(void *);

// === utils ===

int get_run();
void set_run(int run_new);

double get_time();

int gpib_open(const char *name);
int gpib_close(int dev);
int gpib_write(int dev, const char *str);
int gpib_read(int dev, char *buf, size_t buf_length);
void gpib_print_error(int dev);

int usbtmc_open(const char *name);
int usbtmc_close(int dev);
int usbtmc_write(int dev, const char *str);
int usbtmc_read(int dev, char *buf, size_t buf_length);

int dev_print(int dev, const char *format, ...);

#ifdef DEV_AKIP_TMC
	#define dev_open  usbtmc_open
	#define dev_close usbtmc_close
	#define dev_read  usbtmc_read
	#define dev_write usbtmc_write
#else
	#define dev_open  gpib_open
	#define dev_close gpib_close
	#define dev_read  gpib_read
	#define dev_write gpib_write
#endif

// === global variables ===
char dir_str[100];
pthread_rwlock_t run_lock;
int run;
char filename_vac[100];
const char *experiment_name;

// === program entry point ===
int main(int argc, char const *argv[])
{
	int ret = 0;
	int status;

	time_t start_time;
	struct tm start_time_struct;

	pthread_t t_commander;
	pthread_t t_worker;

	// === check input parameters
	if (argc < 2)
	{
		fprintf(stderr, "# E: Usage: vac <experiment_name>\n");
		ret = -1;
		goto main_exit;
	}
	experiment_name = argv[1];

	// === get start time of experiment
	start_time = time(NULL);
	localtime_r(&start_time, &start_time_struct);

	// === we need actual information w/o buffering
	setlinebuf(stdout);
	setlinebuf(stderr);

	// === initialize run state variable
	pthread_rwlock_init(&run_lock, NULL);
	run = 1;

	// === create dirictory in "20191012_153504_<experiment_name>" format
	snprintf(dir_str, 100, "%04d-%02d-%02d_%02d-%02d-%02d_%s",
		start_time_struct.tm_year + 1900,
		start_time_struct.tm_mon + 1,
		start_time_struct.tm_mday,
		start_time_struct.tm_hour,
		start_time_struct.tm_min,
		start_time_struct.tm_sec,
		experiment_name
	);
	status = mkdir(dir_str, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (status == -1)
	{
		fprintf(stderr, "# E: unable to create experiment directory (%s)\n", strerror(errno));
		ret = -2;
		goto main_exit;
	}

	// === create file names
	snprintf(filename_vac, 100, "%s/vac.dat", dir_str);
	// printf("filename_vac \"%s\"\n", filename_vac);

	// === now start threads
	pthread_create(&t_commander, NULL, commander, NULL);
	pthread_create(&t_worker, NULL, worker, NULL);

	// === and wait ...
	pthread_join(t_worker, NULL);

	// === cancel commander thread becouse we don't need it anymore
	// === and wait for cancelation finish
	pthread_cancel(t_commander);
	pthread_join(t_commander, NULL);

	printf("\n");

	main_exit:
	return ret;
}

// === commander function
void *commander(void *arg)
{
	(void) arg;

	char str[100];
	char *s;
	int ccount;

	while(get_run())
	{
		printf("> ");

		s = fgets(str, 100, stdin);
		if (s == NULL)
		{
			fprintf(stderr, "# E: Exit\n");
			set_run(0);
			break;
		}

		switch(str[0])
		{
			case 'h':
				printf(
					"Help:\n"
					"\th -- this help;\n"
					"\tq -- exit the program;\n");
				break;
			case 'q':
				set_run(0);
				break;
			default:
				ccount = strlen(str)-1;
				fprintf(stderr, "# E: Unknown command (%.*s)\n", ccount, str);
				break;
		}
	}

	return NULL;
}

// === worker function
void *worker(void *arg)
{
	(void) arg;

	int r;

	int osc_fd;
	int pps_fd;
	int vm_fd;

	int    vac_index;
	double vac_time;
	double pps_voltage;
	double pps_current;
	double vm_voltage;
	double laser_voltage;
	double laser_current;

	double voltage;

	FILE  *vac_fp;
	FILE  *gp;
	char   buf[100];

	// === first we have to connect to instruments
	r = usbtmc_open(OSC_NAME);
	if(r == -1) goto worker_osc_close;
	osc_fd = r;

	r = dev_open(PPS_NAME);
	if(r == -1) goto worker_pps_open;
	pps_fd = r;

	r = dev_open(VM_NAME);
	if(r == -1) goto worker_vm_open;
	vm_fd = r;

	// === init pps
	dev_write(pps_fd, "output 0");
	dev_write(pps_fd, "instrument:nselect 1");
	dev_print(pps_fd, "voltage:limit %dV", ((int)VOLTAGE_MAX) + 1);
	dev_write(pps_fd, "voltage 0.0");
	dev_print(pps_fd, "current %.1lf", CURRENT_MAX);
	dev_write(pps_fd, "channel:output 1");
	dev_write(pps_fd, "instrument:nselect 2");
	dev_print(pps_fd, "voltage:limit %.1lfV", LASER_VOLTAGE + 0.5);
	dev_print(pps_fd, "voltage %.1lf", LASER_VOLTAGE);
	dev_print(pps_fd, "current %.2lf", LASER_CURRENT);
	dev_write(pps_fd, "channel:output 1");
	dev_write(pps_fd, "instrument:nselect 1");
	// gpib_print_error(pps_fd);

	// === init vm
	dev_write(vm_fd, "function \"voltage:ac\"");
	dev_write(vm_fd, "voltage:ac:range:auto on");
	dev_write(vm_fd, "trigger:source immediate");
	dev_write(vm_fd, "trigger:delay:auto off");
	dev_write(vm_fd, "trigger:delay 0");
	dev_write(vm_fd, "trigger:count 1");
	dev_write(vm_fd, "sample:count 1");
	// gpib_print_error(vm_fd);

	// === init osc
	usbtmc_write(osc_fd, "dds:switch 0");
	usbtmc_write(osc_fd, "dds:type square");
	usbtmc_write(osc_fd, "dds:freq 500");
	usbtmc_write(osc_fd, "dds:amp 3.5");
	usbtmc_write(osc_fd, "dds:offset 1.75");
	usbtmc_write(osc_fd, "dds:duty 50");
	usbtmc_write(osc_fd, "dds:wave:mode off");
	usbtmc_write(osc_fd, "dds:burst:switch off");
	usbtmc_write(osc_fd, "dds:switch 1");

	// === create vac file
	vac_fp = fopen(filename_vac, "w+");
	if(vac_fp == NULL)
	{
		fprintf(stderr, "# E: unable to open file \"%s\" (%s)\n", filename_vac, strerror(errno));
		goto worker_vac_fopen;
	}
	setlinebuf(vac_fp);

	// === write vac header
	fprintf(vac_fp, "# Dependence of alternative voltage on voltage using resistive divider\n");
	fprintf(vac_fp, "# Experiment name \"%s\"\n", experiment_name);
	fprintf(vac_fp, "# 1: index\n");
	fprintf(vac_fp, "# 2: time, s\n");
	fprintf(vac_fp, "# 3: pps voltage, V\n");
	fprintf(vac_fp, "# 4: pps current, A\n");
	fprintf(vac_fp, "# 5: vm current, A\n");
	fprintf(vac_fp, "# 6: laser voltage, V\n");
	fprintf(vac_fp, "# 7: laser current, A\n");
	fprintf(vac_fp, "# 8: laset modulation rate, Hz\n");
	fprintf(vac_fp, "# 9: laset modulation duty, %%\n");

	// === open gnuplot
	snprintf(buf, 100, "gnuplot > %s/gnuplot.log 2>&1", dir_str);
	gp = popen(buf, "w");
	if (gp == NULL)
	{
		fprintf(stderr, "# E: unable to open gnuplot pipe (%s)\n", strerror(errno));
		goto worker_gp_popen;
	}
	setlinebuf(gp);

	// === prepare gnuplot
	fprintf(gp, "set xrange [0:10]\n");
	fprintf(gp, "set xlabel \"Voltage, V\"\n");
	fprintf(gp, "set ylabel \"Voltage (AC), V\"\n");

	// === let the action begins!
	vac_index = 0;

	while(get_run())
	{
		voltage = vac_index * VOLTAGE_STEP;
		if (voltage > VOLTAGE_MAX)
		{
			set_run(0);
			break;
		}

		dev_print(pps_fd, "voltage %.3lf", voltage);

		usleep(STEP_DELAY);

		vac_time = get_time();

		dev_write(pps_fd, "measure:voltage:all?");
		dev_read(pps_fd, buf, 100);
		sscanf(buf, "%lf, %lf", &pps_voltage, &laser_voltage);

		dev_write(pps_fd, "measure:current:all?");
		dev_read(pps_fd, buf, 100);
		sscanf(buf, "%lf, %lf", &pps_current, &laser_current);

		dev_write(vm_fd, "read?");
		dev_read(vm_fd, buf, 100);
		vm_voltage = atof(buf);

		fprintf(vac_fp, "%d\t%le\t%.3le\t%.3le\t%.8le\t%.3le\t%.3le\t%.1lf\t%d\n",
			vac_index, vac_time, pps_voltage, pps_current, vm_voltage,
			laser_voltage, laser_current, 500.0, 50
		);

		fprintf(gp,
			"set title \"i = %d, t = %.3lf s, Ul = %.3lf V, Il = %.3lf A, freq = %.1lf Hz, duty = %d %%\"\n"
			"plot \"%s\" u 3:5 w l lw 1 title \"U = %.3lf V, Vac = %.3le V\"\n",
			vac_index, vac_time, laser_voltage, laser_current, 500.0, 50,
			filename_vac, pps_voltage, vm_voltage
		);

		vac_index++;
	}

	dev_write(pps_fd, "output 0");
	dev_write(pps_fd, "voltage 0");

	usbtmc_write(osc_fd, "dds:switch 0");
	usbtmc_write(osc_fd, "dds:offset 0");

	dev_write(pps_fd, "system:beeper");

	r = pclose(gp);
	if (r == -1)
	{
		fprintf(stderr, "# E: Unable to close gnuplot pipe (%s)\n", strerror(errno));
	}
	worker_gp_popen:

	r = fclose(vac_fp);
	if (r == EOF)
	{
		fprintf(stderr, "# E: Unable to close file \"%s\" (%s)\n", filename_vac, strerror(errno));
	}
	worker_vac_fopen:

	dev_close(vm_fd);
	worker_vm_open:

	dev_close(pps_fd);
	worker_pps_open:

	usbtmc_close(osc_fd);
	worker_osc_close:

	return NULL;
}

// === UTILS ===

int get_run()
{
	int run_local;
	pthread_rwlock_rdlock(&run_lock);
		run_local = run;
	pthread_rwlock_unlock(&run_lock);
	return run_local;
}

void set_run(int run_new)
{
	pthread_rwlock_wrlock(&run_lock);
		run = run_new;
	pthread_rwlock_unlock(&run_lock);
}

double get_time()
{
	static int first = 1;
	static struct timeval t_first = {0};
	struct timeval t = {0};
	double ret;
	int r;

	if (first == 1)
	{
		r = gettimeofday(&t_first, NULL);
		if (r == -1)
		{
			fprintf(stderr, "# E: unable to get time (%s)\n", strerror(errno));
			ret = -1;
		}
		else
		{
			ret = 0.0;
			first = 0;
		}
	}
	else
	{
		r = gettimeofday(&t, NULL);
		if (r == -1)
		{
			fprintf(stderr, "# E: unable to get time (%s)\n", strerror(errno));
			ret = -2;
		}
		else
		{
			ret = (t.tv_sec - t_first.tv_sec) * 1e6 + (t.tv_usec - t_first.tv_usec);
			ret /= 1e6;
		}
	}

	return ret;
}

// === GPIB ===

int gpib_open(const char *name)
{
	int r;

	r = ibfind(name);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to open gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}

	return r;
}

int gpib_close(int dev)
{
	int r;
	int ret = 0;

	r = ibclr(dev);
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to clr gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}

	r = gpib_write(dev, "*rst");
	if (r == -1)
	{
		ret = r;
	}

	sleep(1);

	r = ibloc(dev);
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to loc gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}

	return ret;
}

int gpib_write(int dev, const char *str)
{
	int r;
	int ret = 0;

	r = ibwrt(dev, str, strlen(str));
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to write to gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}
	else
	{
		ret = ibcnt;
	}

	return ret;
}

int gpib_read(int dev, char *buf, size_t buf_length)
{
	int r;
	int ret = 0;

	r = ibrd(dev, buf, buf_length);
	if (r & 0x8000)
	{
		ret = -1;
		fprintf(stderr, "# E: unable to write to gpib (ibsta = %d, iberr = %d)\n", ibsta, iberr);
	}
	else
	{
		ret = ibcnt;
		if (ibcnt < buf_length)
		{
			buf[ibcnt] = 0;
		}
	}

	return ret;
}

void gpib_print_error(int dev)
{
	char buf[100] = {0};
	gpib_write(dev, "system:error?");
	gpib_read(dev, buf, 100);
	fprintf(stderr, "# [debug] error = %s\n", buf);
}

// === USBTMC ===

int usbtmc_open(const char *name)
{
	int r;

	r = open(name, O_RDWR);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to open usbtmc \"%s\" (%s)\n", name, strerror(errno));
	}

	return r;
}

int usbtmc_close(int dev)
{
	int r;

	usbtmc_write(dev, "*rst");

	r = close(dev);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to close usbtmc (%s)\n", strerror(errno));
	}

	return r;
}

int usbtmc_write(int dev, const char *str)
{
	int r;

	r = write(dev, str, strlen(str));
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to write to usbtmc \"%s\" (%s)\n", str, strerror(errno));
	}

	return r;
}

int usbtmc_read(int dev, char *buf, size_t buf_length)
{
	int r;

	r = read(dev, buf, buf_length);
	if (r == -1)
	{
		fprintf(stderr, "# E: unable to read from usbtmc (%s)\n", strerror(errno));
	}

	return r;
}

int dev_print(int dev, const char *format, ...)
{
	int r;
	va_list args;
	char buf[100];
	const size_t bufsize = 100;

	va_start(args, format);
	r = vsnprintf(buf, bufsize, format, args);
	if (r < 0)
	{
		fprintf(stderr, "# E: unable to printf to buffer (%s)\n", strerror(errno));
		goto usbtmc_print_vsnprintf;
	}
	r = dev_write(dev, buf);
	usbtmc_print_vsnprintf:
	va_end(args);

	return r;
}
