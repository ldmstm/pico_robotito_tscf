#ifndef WIFI_CONNECTION_H
#define WIFI_CONNECTION_H

void wifi_connect (const char *ssid, const char *password);

void wifi_wait_connected (void);

#endif /* WIFI_CONNECTION_H */
