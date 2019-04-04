/*
 *  smart_power_2_mm.c
 *
 * 	PMCTrack monitoring module enabling to obtain power measurements
 *  with Odroid Smart Power 2
 *
 *  Copyright (c) 2015 Juan Carlos Saez <jcsaezal@ucm.es>
 *                2019 Germán Franco Dorca <germanfr@ucm.es>
 *
 *  This code is licensed under the GNU GPL v2.
 */

#include <pmc/hl_events.h>
#include <pmc/mc_experiments.h>
#include <pmc/monitoring_mod.h>
#include <pmc/smart_power_2.h>

#define SPOWER2_MODULE_STR "Odroid Smart Power 2"

/* Per-thread private data for this monitoring module */
typedef struct {
	struct spower2_sample last_sample;
	unsigned long time_last_sample;
	int security_id;
} spower2_thread_data_t;

enum {SPOWER2_POWER=0,SPOWER2_CURRENT,SPOWER2_ENERGY,SPOWER2_NR_MEASUREMENTS};

static int initialize_system_wide_spower2_structures(void);


/* Return the capabilities/properties of this monitoring module */
static void spower2_module_counter_usage(monitoring_module_counter_usage_t* usage)
{
	usage->hwpmc_mask=0;
	usage->nr_virtual_counters=SPOWER2_NR_MEASUREMENTS;
	usage->nr_experiments=0;
	usage->vcounter_desc[SPOWER2_POWER]="power_mw";
	usage->vcounter_desc[SPOWER2_CURRENT]="current_ma";
	usage->vcounter_desc[SPOWER2_ENERGY]="energy_uj";
}

/* MM initialization */
static int spower2_enable_module(void)
{
	int retval=0;

	if ((retval=initialize_system_wide_spower2_structures())) {
		printk(KERN_INFO "Couldn't initialize system-wide power structures");
		return retval;
	}

	if ((retval=spower2_start_measurements()))
		return retval;

	return 0;
}

/* MM cleanup function */
static void spower2_disable_module(void)
{
	spower2_stop_measurements();
	printk(KERN_ALERT "%s monitoring module unloaded!!\n",SPOWER2_MODULE_STR);
}


/* Display configuration parameters */
static int spower2_on_read_config(char* str, unsigned int len)
{
	char* dst=str;

	dst+=sprintf(dst,"spower2_sampling_period = %u\n",spower2_get_sampling_period());
	dst+=sprintf(dst,"spower2_cummulative_energy = %llu\n",spower2_get_energy_count());

	return dst - str;
}

/* Change configuration parameters */
static int spower2_on_write_config(const char *str, unsigned int len)
{
	int val;
	int ret;

	if (sscanf(str,"spower2_sampling_period %d",&val)==1) {
		if ((ret=spower2_set_sampling_period(val)))
			return ret;
		return len;
	} else if (strncmp(str,"reset_energy_count",18)==0) {
		spower2_reset_energy_count();
	}
	return 0;
}

/* on fork() callback */
static int spower2_on_fork(unsigned long clone_flags, pmon_prof_t* prof)
{
	spower2_thread_data_t*  data= NULL;

	if (prof->monitoring_mod_priv_data!=NULL)
		return 0;


	data= kmalloc(sizeof (spower2_thread_data_t), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	memset(&data->last_sample,0,sizeof(struct spower2_sample));
	data->time_last_sample=jiffies;
	data->security_id=current_monitoring_module_security_id();
	prof->monitoring_mod_priv_data = data;
	return 0;
}

/*
 * Update cumulative energy counters in the thread structure and
 * set the associated virtual counts in the PMC sample structure
 */
static int spower2_on_new_sample(pmon_prof_t* prof,int cpu,pmc_sample_t* sample,int flags,void* data)
{
	spower2_thread_data_t* tdata=prof->monitoring_mod_priv_data;
	int i=0;
	int cnt_virt=0;

	if (tdata==NULL || prof->virt_counter_mask==0)
		return 0;

	/* dump data if we got something */
	if (spower2_get_sample(tdata->time_last_sample,&tdata->last_sample)==0)
		return 0;

	tdata->time_last_sample=jiffies;

	/* Embed virtual counter information so that the user can see what's going on */
	for (i=0; i<SPOWER2_NR_MEASUREMENTS; i++) {
		if ((prof->virt_counter_mask & (1<<i)) ) {
			switch (i) {
			case SPOWER2_POWER:
				sample->virtual_counts[cnt_virt++]=tdata->last_sample.m_watt;
				break;
			case SPOWER2_CURRENT:
				sample->virtual_counts[cnt_virt++]=tdata->last_sample.m_ampere;
				break;
			case SPOWER2_ENERGY:
				sample->virtual_counts[cnt_virt++]=tdata->last_sample.m_ujoules;
				break;
			default:
				continue;
			}

			sample->virt_mask|=(1<<i);
			sample->nr_virt_counts++;
		}
	}

	return 0;
}

/* Free up private data */
static void spower2_on_free_task(pmon_prof_t* prof)
{
	if (prof->monitoring_mod_priv_data)
		kfree(prof->monitoring_mod_priv_data);
}

/* Support for system-wide power measurement (Reuse per-thread info as is) */
static DEFINE_PER_CPU(spower2_thread_data_t, cpu_syswide);

/* Initialize resources to support system-wide monitoring with Intel RAPL */
static int initialize_system_wide_spower2_structures(void)
{
	int cpu;
	spower2_thread_data_t* data;

	for_each_possible_cpu(cpu) {
		data=&per_cpu(cpu_syswide, cpu);

		memset(&data->last_sample,0,sizeof(struct spower2_sample));
		data->time_last_sample=jiffies;

		/* These fields are not used by the system-wide monitor
			Nevertheless we initialize them both just in case...
		*/
		data->security_id=current_monitoring_module_security_id();
	}

	return 0;
}

/*	Invoked on each CPU when starting up system-wide monitoring mode */
static int spower2_on_syswide_start_monitor(int cpu, unsigned int virtual_mask)
{
	spower2_thread_data_t* data;

	/* Probe only */
	if (cpu==-1) {
		/* Make sure virtual_mask only has 1s in the right bits
			- This can be checked easily
				and( virtual_mask,not(2^nr_available_vcounts - 1)) == 0
		*/
		if (virtual_mask & ~((1<<SPOWER2_NR_MEASUREMENTS)-1))
			return -EINVAL;
		else
			return 0;
	}

	data=&per_cpu(cpu_syswide, cpu);

	memset(&data->last_sample,0,sizeof(struct spower2_sample));
	data->time_last_sample=jiffies;

	return 0;
}

/*	Invoked on each CPU when stopping system-wide monitoring mode */
static void spower2_on_syswide_refresh_monitor(int cpu, unsigned int virtual_mask)
{
	/* Do nothing (for now) */
}

/* 	Dump virtual-counter values for this CPU */
static void spower2_on_syswide_dump_virtual_counters(int cpu, unsigned int virtual_mask,pmc_sample_t* sample)
{
	spower2_thread_data_t* data=&per_cpu(cpu_syswide, cpu);
	int i=0;
	int cnt_virt=0;

	if (!virtual_mask)
		return;

	/* dump data if we got something */
	if (spower2_get_sample(data->time_last_sample,&data->last_sample)==0)
		return;

	data->time_last_sample=jiffies;

	/* Embed virtual counter information so that the user can see what's going on */
	for (i=0; i<SPOWER2_NR_MEASUREMENTS; i++) {
		if ((virtual_mask & (1<<i)) ) {
			switch (i) {
			case SPOWER2_POWER:
				sample->virtual_counts[cnt_virt++]=data->last_sample.m_watt;
				break;
			case SPOWER2_CURRENT:
				sample->virtual_counts[cnt_virt++]=data->last_sample.m_ampere;
				break;
			case SPOWER2_ENERGY:
				sample->virtual_counts[cnt_virt++]=data->last_sample.m_ujoules;
				break;
			default:
				continue;
			}

			sample->virt_mask|=(1<<i);
			sample->nr_virt_counts++;
		}
	}
}

/* Implementation of the monitoring_module_t interface */
monitoring_module_t spower2_mm= {
	.info=SPOWER2_MODULE_STR,
	.id=-1,
	.enable_module=spower2_enable_module,
	.disable_module=spower2_disable_module,
	.on_read_config=spower2_on_read_config,
	.on_write_config=spower2_on_write_config,
	.on_fork=spower2_on_fork,
	.on_new_sample=spower2_on_new_sample,
	.on_free_task=spower2_on_free_task,
	.module_counter_usage=spower2_module_counter_usage,
	.on_syswide_start_monitor=spower2_on_syswide_start_monitor,
	.on_syswide_refresh_monitor=spower2_on_syswide_refresh_monitor,
	.on_syswide_dump_virtual_counters=spower2_on_syswide_dump_virtual_counters
};
