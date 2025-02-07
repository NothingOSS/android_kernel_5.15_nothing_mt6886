#ifndef _TOUCHPANEL_EVENT_NOTIFY_H
#define _TOUCHPANEL_EVENT_NOTIFY_H

struct touchpanel_coordinate {
    int x;//x coordinate
    int y;//y coordinate
};

typedef enum {
    TOUCHPANEL_FPEVENT_UP,
    TOUCHPANEL_FPEVENT_DOWN,
}touchpanel_event_status;

int touchpanel_event_register_notifier(struct notifier_block *nb);
int touchpanel_event_unregister_notifier(struct notifier_block *nb);

/* callee API */
int touchpanel_event_call_notifier(unsigned long val, void *data);

#endif	/* _TOUCHPANEL_EVENT_NOTIFY_H */
