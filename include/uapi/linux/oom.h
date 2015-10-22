#ifndef _UAPI__INCLUDE_LINUX_OOM_H
#define _UAPI__INCLUDE_LINUX_OOM_H

/*
 * /proc/<pid>/oom_score_adj set to OOM_SCORE_ADJ_MIN disables oom killing for
 * pid.
 */
#define OOM_SCORE_ADJ_MIN	(-1000)
#define OOM_SCORE_ADJ_MAX	1000

/*
 * /proc/<pid>/oom_adj set to -17 protects from the oom killer for legacy
 * purposes.
 */
#define OOM_DISABLE (-17)
/* inclusive */
#define OOM_ADJUST_MIN (-16)
#define OOM_ADJUST_MAX 15

#define OOM_QOS_FLAGS_ENDURANCE_OFFSET 0
#define OOM_QOS_FLAGS_ENDURANCE_MAX 120 /* Currently, we limite the maximum of
                                         * QOS endurance to 120 seconds. */

#define OOM_QOS_FLAGS_PREFER_KILL_OFFSET 8
#define get_oom_qos_endurance(r, f)                                     \
    do {                                                                \
        r = f & (0xff << OOM_QOS_FLAGS_ENDURANCE_OFFSET);               \
        if (r > OOM_QOS_FLAGS_ENDURANCE_MAX)                            \
            r = OOM_QOS_FLAGS_ENDURANCE_MAX;                            \
        if (r < 0)                                                      \
            r = 0;                                                      \
    } while(0)

#define is_prefer_to_kill(f) ((f & (1 << OOM_QOS_FLAGS_PREFER_KILL_OFFSET)) != 0)
#endif /* _UAPI__INCLUDE_LINUX_OOM_H */
