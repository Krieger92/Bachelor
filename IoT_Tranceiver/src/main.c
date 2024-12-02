#include <stdio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/spsc_lockfree.h>
#include <zephyr/drivers/flash.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/sntp.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/net/net_if.h>
#include <time.h>

// #include <zephyr/net/mqtt.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
// #include <zephyr/net/tls_credentials.h>

#include "net_sample_common.h"



#define SNTP_SERVER		("216.239.35.4") // Google's public NTP server time.google.com


int err, ret;
const struct device *modem = DEVICE_DT_GET(DT_ALIAS(modem));
struct sntp_time ts;
struct tm *tm_info;
struct sockaddr_in sntp_server;

//add an array for storing influxdb line protocol messages
// #define MAX_MESSAGES 1
// #define MAX_MESSAGE_LENGTH 1
// char influxdb_messages[MAX_MESSAGES][MAX_MESSAGE_LENGTH];
// int message_count = 0, messageID = 1;



//MQTT
// #define MQTT_CLIENT_ID "zephyr_publisher"
// #define MQTT_BROKER_ADDR "98.64.193.248"
// #define MQTT_BROKER_PORT 1883
// #define MQTT_TOPIC "telegraf_mqtt/testpm_iothub10"
// #define MQTT_CLIENT_USER "makeen"
// #define MQTT_CLIENT_PASS "jOkhnBaaTURj93N"
// #define APP_MQTT_BUFFER_SIZE	128
// #define APP_CONNECT_TRIES	10
// #define APP_CONNECT_TIMEOUT_MS	2000
// #define APP_SLEEP_MSECS		500
// #define CONFIG_NET_SAMPLE_APP_MAX_ITERATIONS 500
// #define CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS 0

// #define APP_BMEM
// #define APP_DMEM
// #define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")
// #define PRINT_RESULT(func, rc) \
// 	LOG_INF("%s: %d <%s>", (func), rc, RC_STR(rc))

// static struct sockaddr_storage broker;
// static APP_BMEM struct mqtt_client client_ctx;
// static uint8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
// static uint8_t tx_buffer[APP_MQTT_BUFFER_SIZE];
// static APP_BMEM struct pollfd fds[1];
// static APP_BMEM int nfds;
// static APP_BMEM bool connected;

// #define APP_CA_CERT_TAG 1
// #define APP_CERT_TAG 2
// #define APP_KEY_TAG 3

// const char* ca_certificate = "-----BEGIN CERTIFICATE-----\n" \
// "MIIDrDCCApSgAwIBAgIULf3x3PRvbcHwF/imcDtDmW9gfT0wDQYJKoZIhvcNAQEL\n" \
// "BQAwZzELMAkGA1UEBhMCQ0ExEDAOBgNVBAgMB0FsYmVydGExETAPBgNVBAcMCEVk\n" \
// "bW9udG9uMRQwEgYDVQQKDAtFeGFtcGxlLmNvbTELMAkGA1UECwwCQ0ExEDAOBgNV\n" \
// "BAMMB2NhLm1xdHQwHhcNMjQwMzA1MDkzMTI5WhcNMjkwMzA1MDkzMTI5WjBnMQsw\n" \
// "CQYDVQQGEwJDQTEQMA4GA1UECAwHQWxiZXJ0YTERMA8GA1UEBwwIRWRtb250b24x\n" \
// "FDASBgNVBAoMC0V4YW1wbGUuY29tMQswCQYDVQQLDAJDQTEQMA4GA1UEAwwHY2Eu\n" \
// "bXF0dDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKhplpHiJnaBEmdA\n" \
// "FDwPCMMp7cUO+okX/79ZLD/pWFUwOZ2gAW9jk+vNwchp/9J8MAk/W+VcevD703ot\n" \
// "1MTuhAbSw4vBvtSHP0LCpr5COoeHE74Jqdc8FL7pvYZJCyMvSVkDp70G7PA8X3t9\n" \
// "/nSAI+IbKgBW8+YvFZ6Av26TM2FCAn39EijdQJziM+revebPurhUM92Y1o0FabYW\n" \
// "xUEXXZ6/FO3t3/HIdGWiST1+cC/mV4d6Yt7R5U82LuqqVPEA/7lcUorNT+IE1Pan\n" \
// "35fX0J9HYNAOTQmcgS6gmcB3l08zRv4mKVmkNQEDXipfB9omFHfDbMo9+Oi2GZxt\n" \
// "xgZGyCcCAwEAAaNQME4wHQYDVR0OBBYEFFbFbEj7eaxVMVlyy/I2Sx0pHK+lMB8G\n" \
// "A1UdIwQYMBaAFFbFbEj7eaxVMVlyy/I2Sx0pHK+lMAwGA1UdEwQFMAMBAf8wDQYJ\n" \
// "KoZIhvcNAQELBQADggEBAG1ehEcxHBm5BdE44BJEg2FziXH6RAUDWw71iWAdWfVy\n" \
// "d6rdh7mRxL3ZapVFuw56X3bASnANJgkRB12qSQseLWLepIl4mYDtJlTYDlJRKYof\n" \
// "NY7kQ+b2JS7ux/ey/zD2X0iZp0D02qsbrVl9UWUdgGKot9XmoClkYynhCXFol+MZ\n" \
// "s1KW+jUMtG6CH0BEdgcFOtcD1KRxtlIcscQ871BFxyga3YH2mgIt/etYVkUal1Kg\n" \
// "Cf/rHOuKtnWWWHKn7uT42/KEAaiYNvFvfq3eFu+E1e1Vez3TjXCawgEzLCiCYWeT\n" \
// "PhK8RL7z0g1CxeygEMLpxfSk2mBhN2yJxuyrfYqM/MA=\n" \
// "-----END CERTIFICATE-----\n";
// const char* client_certificate = "-----BEGIN CERTIFICATE-----\n" \
// "MIIDnDCCAoSgAwIBAgIBATANBgkqhkiG9w0BAQsFADBnMQswCQYDVQQGEwJDQTEQ\n" \
// "MA4GA1UECAwHQWxiZXJ0YTERMA8GA1UEBwwIRWRtb250b24xFDASBgNVBAoMC0V4\n" \
// "YW1wbGUuY29tMQswCQYDVQQLDAJDQTEQMA4GA1UEAwwHY2EubXF0dDAeFw0yNDAz\n" \
// "MjAwODAyMDZaFw0zNDAzMTgwODAyMDZaMHExCzAJBgNVBAYTAkNBMRAwDgYDVQQI\n" \
// "DAdBbGJlcnRhMREwDwYDVQQHDAhFZG1vbnRvbjEUMBIGA1UECgwLRXhhbXBsZS5j\n" \
// "b20xDzANBgNVBAsMBk1ha2VlbjEWMBQGA1UEAwwNdGVsZWdyYWYubXF0dDCCASIw\n" \
// "DQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALc1wwFbx1C/P+r/gKJm0SEG0GDv\n" \
// "xEyRdJH41FxJ6GZhshJF91MN69Zja3vztkxIf/fxw+ZOZuDRcJKP3AHpudAah5RB\n" \
// "4oBhUPCEr0CLU9Asy6QvBIWU+zwhhQnr7NKeNLGnRFqYXdG2xt7Wsa61QDWFvBgt\n" \
// "a2Zf1cFdzrrUaYHkp2KCW6mc7T2XWu3IHm81cBzagrfCk6nma9CbM09wfq+Mm6n1\n" \
// "AqSm2j4PZ4WQBkwMG/r9b+cqfvDK41JdL0nNTfFPxve0XDcLgLjXaoO2bYlhswPO\n" \
// "/pGClMf8rWM5GqOcP+KTyfnakI2HdI7shO5EVbX067CoJnctznpc6CJxGB8CAwEA\n" \
// "AaNJMEcwCwYDVR0PBAQDAgQwMBMGA1UdJQQMMAoGCCsGAQUFBwMBMBgGA1UdEQQR\n" \
// "MA+CDXRlbGVncmFmLm1xdHQwCQYDVR0TBAIwADANBgkqhkiG9w0BAQsFAAOCAQEA\n" \
// "AV6o3VKhAxp+fIwNMqgL9434Y8AT/mQY5I4R2tE+6RAB8LXRPeyauEjJiLwXwexS\n" \
// "THw8gQJnfIFZTeNghdMmknacRy21Gneh06pdoPPE+Paa3y73cvsuxCn5ZTk0Pdcn\n" \
// "UA/ksFKHGeDuokz/NRGSVOYN1zjN6SGO9FlAPuzvstMFRKn0HDLFhgfNYloHrhtS\n" \
// "JqfwTMtQVxEaBfr7l3aJFid1smv7QgyOsp7XZHeh5gz845svnHQruJCLNIkIdQq5\n" \
// "P2HtDsTr+vhTIZIt9fgJvWVp5tactr6jw2y0mP9lEpyE5YRaf9V8UwvhqZCRcMii\n" \
// "fRnpbUo+KCqDK9eVnP5mPQ==\n" \
// "-----END CERTIFICATE-----\n";
// const char* client_key = "-----BEGIN RSA PRIVATE KEY-----\n" \
// "MIIEpAIBAAKCAQEAtzXDAVvHUL8/6v+AombRIQbQYO/ETJF0kfjUXEnoZmGyEkX3\n" \
// "Uw3r1mNre/O2TEh/9/HD5k5m4NFwko/cAem50BqHlEHigGFQ8ISvQItT0CzLpC8E\n" \
// "hZT7PCGFCevs0p40sadEWphd0bbG3taxrrVANYW8GC1rZl/VwV3OutRpgeSnYoJb\n" \
// "qZztPZda7cgebzVwHNqCt8KTqeZr0JszT3B+r4ybqfUCpKbaPg9nhZAGTAwb+v1v\n" \
// "5yp+8MrjUl0vSc1N8U/G97RcNwuAuNdqg7ZtiWGzA87+kYKUx/ytYzkao5w/4pPJ\n" \
// "+dqQjYd0juyE7kRVtfTrsKgmdy3OelzoInEYHwIDAQABAoIBAQCTR3i4Atu7ocL0\n" \
// "WulBpH3BmwSwqI6ZLuUS44Fzn0Zf53GLFwu3jJ1l/w5M2qJVuIKHA6eD4Mr7N+LT\n" \
// "Lyqhrw5cqLcNC9nVcmXqs/9eV6LM2vXpypJMAHPm5ttYGnyK4/tgRSLB4XKqP9Cn\n" \
// "GthDphkz7YpDtV99MNVZWNvzfQZED/up64Fkdo8Hej2nR8KBp0ro+baQFwPg8+jl\n" \
// "HWnf0uuCWR4Efo3H6azKroujOvpgBIdnqsfNeadmne7h09PwpH56c45osXzFGEy3\n" \
// "T7lzqXl2/+E7GY/XMYm/TTct/rjoYbm2Bh57DI4p6j1tBmvHVTUBsFT+Vy4WZWUU\n" \
// "ev4oRjXBAoGBAOIiOaQuaqb/qTjVuct8iAp3pD6GpDYRxyoGjbM5FMqGUZGl+RGq\n" \
// "9eSVtDf+8AlVGYBCMlNPjY3ujTb0bQm/lVmwKCvuzf8/R3o+EyjId2fDHJbPWRas\n" \
// "f28Or4qHj8IuM8WV6e5ID/3NU03z4b1qvw1j6NmWZd1yPOZpfQ9xnfEXAoGBAM9o\n" \
// "P/RCf/wK2DQB1T/Vujj1j12+Bs783P0Oa2xQc1KQQt7M0fdvjsRiT0xwYHEkb0wR\n" \
// "zXdSPJIASm+W+LCvPkZ6BDURpZfZKtEgu+UyUfy78UxavLf6xk5cnO5THB/OH6Ny\n" \
// "s6vV97MA7P8YMgSdMQ/XzblwZroTF+sbvTeLiyY5AoGBAMDiiqI1iXlq54/4FeOx\n" \
// "FGVBShKfTCBmp6h8MI/oJ4jca2U92ias2dMhsrh0t5BuSzMww7Z2S/r9uOxgL0W6\n" \
// "ag03ifx3VLTc5/4oRrVp0RfrwE4PDSGe/4/eSfNy0DfEqxplVkWOqD1Hzz/Vv/6O\n" \
// "t+oXB33ZoIf0MmgjIK50rlD/AoGATRLugTSdHupgwaLUvi4MT9Jg/nL5UL4C2T8c\n" \
// "iFm1IT9eujDe7dZU9YGa+krga+sgOOJ8uhSjUZYuEx3FONNGjyYUniv6n10iXiiN\n" \
// "6nQyp3qoBrnJW40jWEduQdo8n00X0MN1JSwUKj7z0zAIllJbwwls97ZfVlJ2FvZt\n" \
// "8FA423ECgYAeFa8+GtA2Ya2SOCYrxQGH/DGYg/ExY2c6FO8JiiWHa1RGuN/GGbvt\n" \
// "7mJ4UYMdSaK0OLIpp0lQwhy6V3vM9jsRiavKMI+VC2ejyBNf3Ppnu1J6nSEEk79E\n" \
// "aXTAxwENLiruQp9W3UJ3xBjYoP090ld7WjkoMjgLiIfkBMjkZIUHOg==\n" \
// "-----END RSA PRIVATE KEY-----\n";



// const struct device *flash_dev = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

const struct bt_le_scan_param scan_param = BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_FILTER_DUPLICATE, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);
const struct bt_le_scan_param scan_paramRunning = BT_LE_SCAN_PARAM_INIT(BT_LE_SCAN_TYPE_PASSIVE, BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST, BT_GAP_SCAN_FAST_INTERVAL, BT_GAP_SCAN_FAST_WINDOW);


//Function declarations
static char *get_mqtt_message(void);
static bool mess_available();



typedef struct 
{
  uint8_t noIdea[5];
  uint16_t companyID;
  uint8_t frameType;
  uint8_t config[3];
  uint8_t seqID;
  uint16_t counter;
  uint16_t batteryLvl;
  uint16_t adc_val1;
  uint16_t adc_val2;
  bt_addr_le_t addr;
  int8_t rssi;
}__attribute__((packed)) makeen_data_ADC;

typedef struct
{
  uint8_t seqID;
  uint16_t runtimeCounter;
  uint16_t batteryLvl;
  uint16_t adc_val1;
  uint16_t adc_val2;
  bt_addr_le_t addr;
  int8_t rssi;
  uint64_t timestamp;
}__attribute__((packed)) makeen_data_filtered;


SPSC_DEFINE(spscBuf1, makeen_data_ADC, 64);
SPSC_DEFINE(spscBuf2, makeen_data_filtered, 64);


// static void prepare_fds(struct mqtt_client *client)
// {
//   if (client->transport.type == MQTT_TRANSPORT_SECURE) {
// 		fds[0].fd = client->transport.tls.sock;
// 	}

// 	fds[0].events = POLLIN;
// 	nfds = 1;
// }

// static void clear_fds(void)
// {
// 	nfds = 0;
// }

// static int wait(int timeout)
// {
// 	int ret = 0;

// 	if (nfds > 0) {
// 		ret = poll(fds, nfds, timeout);
// 		if (ret < 0) {
// 			LOG_ERR("poll error: %d", errno);
// 		}
// 	}

// 	return ret;
// }

// void mqtt_evt_handler(struct mqtt_client *const client,
// 		      const struct mqtt_evt *evt)
// {
// 	int err;

// 	switch (evt->type) {
// 	case MQTT_EVT_CONNACK:
// 		if (evt->result != 0) {
// 			LOG_ERR("MQTT connect failed %d", evt->result);
// 			break;
// 		}

// 		connected = true;
// 		LOG_INF("MQTT client connected!");

// 		break;

// 	case MQTT_EVT_DISCONNECT:
// 		LOG_INF("MQTT client disconnected %d", evt->result);

// 		connected = false;
// 		clear_fds();

// 		break;

// 	case MQTT_EVT_PUBACK:
// 		if (evt->result != 0) {
// 			LOG_ERR("MQTT PUBACK error %d", evt->result);
// 			break;
// 		}

// 		LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);

// 		break;

// 	case MQTT_EVT_PUBREC:
// 		if (evt->result != 0) {
// 			LOG_ERR("MQTT PUBREC error %d", evt->result);
// 			break;
// 		}

// 		LOG_INF("PUBREC packet id: %u", evt->param.pubrec.message_id);

// 		const struct mqtt_pubrel_param rel_param = {
// 			.message_id = evt->param.pubrec.message_id
// 		};

// 		err = mqtt_publish_qos2_release(client, &rel_param);
// 		if (err != 0) {
// 			LOG_ERR("Failed to send MQTT PUBREL: %d", err);
// 		}

// 		break;

// 	case MQTT_EVT_PUBCOMP:
// 		if (evt->result != 0) {
// 			LOG_ERR("MQTT PUBCOMP error %d", evt->result);
// 			break;
// 		}

// 		LOG_INF("PUBCOMP packet id: %u",
// 			evt->param.pubcomp.message_id);

// 		break;

// 	case MQTT_EVT_PINGRESP:
// 		LOG_INF("PINGRESP packet");
// 		break;

// 	default:
// 		break;
// 	}
// }

// static int publish(struct mqtt_client *client, enum mqtt_qos qos)
// {
//     struct mqtt_publish_param param;

//     char *message = get_mqtt_message();

//     param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
//     param.message.topic.topic.utf8 = (uint8_t *)MQTT_TOPIC;
//     param.message.topic.topic.size = strlen(MQTT_TOPIC);
//     param.message.payload.data = (uint8_t *)message;
//     param.message.payload.len = strlen(message);
//     param.message_id = messageID++;
//     param.dup_flag = 0U;
//     param.retain_flag = 0U;

//     return mqtt_publish(client, &param);
// }

// static int tls_init(void)
// {
// 	int err = -EINVAL;

// #if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
// 	err = tls_credential_add(APP_CA_CERT_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
// 				 ca_certificate, sizeof(ca_certificate));
// 	if (err < 0) {
// 		LOG_ERR("Failed to register public certificate: %d", err);
// 		return err;
// 	}
// #endif

// #if defined(MBEDTLS_KEY_EXCHANGE_SOME_PSK_ENABLED)
// 	err = tls_credential_add(APP_PSK_TAG, TLS_CREDENTIAL_PSK,
// 				 client_psk, sizeof(client_psk));
// 	if (err < 0) {
// 		LOG_ERR("Failed to register PSK: %d", err);
// 		return err;
// 	}

// 	err = tls_credential_add(APP_PSK_TAG, TLS_CREDENTIAL_PSK_ID,
// 				 client_psk_id, sizeof(client_psk_id) - 1);
// 	if (err < 0) {
// 		LOG_ERR("Failed to register PSK ID: %d", err);
// 	}
// #endif

//   err = tls_credential_add(APP_CA_CERT_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
//             ca_certificate, sizeof(ca_certificate));
//   if (err < 0) {
//       printk("Failed to register public certificate: %d", err);
//       return err;
//   }

//   err = tls_credential_add(APP_CERT_TAG, TLS_CREDENTIAL_SERVER_CERTIFICATE,
//             client_certificate, sizeof(client_certificate));
//   if (err < 0) {
//       printk("Failed to register client certificate: %d", err);
//       return err;
//   }

//   err = tls_credential_add(APP_KEY_TAG, TLS_CREDENTIAL_PRIVATE_KEY,
//             client_key, sizeof(client_key));
//   if (err < 0) {
//       printk("Failed to register private key: %d", err);
//       return err;
//   }

// 	return err;
// }

// static void broker_init(void) {
//     struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;
//     broker4->sin_family = AF_INET;
//     broker4->sin_port = htons(MQTT_BROKER_PORT);
//     inet_pton(AF_INET, MQTT_BROKER_ADDR, &broker4->sin_addr);
// }

// static void client_init(struct mqtt_client *client)
// {

//     static struct mqtt_utf8 user_name = {
//         .utf8 = (uint8_t *)MQTT_CLIENT_USER,
//         .size = strlen(MQTT_CLIENT_USER)
//     };

//     static struct mqtt_utf8 password = {
//         .utf8 = (uint8_t *)MQTT_CLIENT_PASS,
//         .size = strlen(MQTT_CLIENT_PASS)
//     };
//     mqtt_client_init(client);

//     broker_init();

//     //MQTT client configuration
//     client->broker = &broker;
//     client->evt_cb = mqtt_evt_handler;
//     client->client_id.utf8 = (uint8_t *)MQTT_CLIENT_ID;
//     client->client_id.size = strlen(MQTT_CLIENT_ID);
//     client->user_name = &user_name;
//     client->password = &password;
//     client->protocol_version = MQTT_VERSION_3_1_1;
//     client->transport.type = MQTT_TRANSPORT_SECURE;
//     // client->keepalive = 60;

//     //MQTT buffers configuration
//     client->rx_buf = rx_buffer;
//     client->rx_buf_size = sizeof(rx_buffer);
//     client->tx_buf = tx_buffer;
//     client->tx_buf_size = sizeof(tx_buffer);

// }

// static int try_to_connect(struct mqtt_client *client)
// {
// 	int rc, i = 0;

// 	while (i++ < APP_CONNECT_TRIES && !connected) {

// 		client_init(client);

// 		rc = mqtt_connect(client);
// 		if (rc != 0) {
// 			PRINT_RESULT("mqtt_connect", rc);
// 			k_sleep(K_MSEC(APP_SLEEP_MSECS));
// 			continue;
// 		}

// 		prepare_fds(client);

// 		if (wait(APP_CONNECT_TIMEOUT_MS)) {
// 			mqtt_input(client);
// 		}

// 		if (!connected) {
// 			mqtt_abort(client);
// 		}
// 	}

// 	if (connected) {
// 		return 0;
// 	}

// 	return -EINVAL;
// }

// static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
// {
// 	int64_t remaining = timeout;
// 	int64_t start_time = k_uptime_get();
// 	int rc;

// 	while (remaining > 0 && connected) {
// 		if (wait(remaining)) {
// 			rc = mqtt_input(client);
// 			if (rc != 0) {
// 				PRINT_RESULT("mqtt_input", rc);
// 				return rc;
// 			}
// 		}

// 		rc = mqtt_live(client);
// 		if (rc != 0 && rc != -EAGAIN) {
// 			PRINT_RESULT("mqtt_live", rc);
// 			return rc;
// 		} else if (rc == 0) {
// 			rc = mqtt_input(client);
// 			if (rc != 0) {
// 				PRINT_RESULT("mqtt_input", rc);
// 				return rc;
// 			}
// 		}

// 		remaining = timeout + start_time - k_uptime_get();
// 	}

// 	return 0;
// }

// #define SUCCESS_OR_EXIT(rc) { if (rc != 0) { return 1; } }
// #define SUCCESS_OR_BREAK(rc) { if (rc != 0) { break; } }

// static int publisher(void)
// {
// 	int i, rc, r = 0;

// 	LOG_INF("attempting to connect: ");
// 	rc = try_to_connect(&client_ctx);
// 	PRINT_RESULT("try_to_connect", rc);
// 	SUCCESS_OR_EXIT(rc);

// 	i = 0;
// 	while (i++ < CONFIG_NET_SAMPLE_APP_MAX_ITERATIONS && connected) {
// 		r = -1;

// 		rc = mqtt_ping(&client_ctx);
// 		PRINT_RESULT("mqtt_ping", rc);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = publish(&client_ctx, MQTT_QOS_0_AT_MOST_ONCE);
// 		PRINT_RESULT("mqtt_publish", rc);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = publish(&client_ctx, MQTT_QOS_1_AT_LEAST_ONCE);
// 		PRINT_RESULT("mqtt_publish", rc);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = publish(&client_ctx, MQTT_QOS_2_EXACTLY_ONCE);
// 		PRINT_RESULT("mqtt_publish", rc);
// 		SUCCESS_OR_BREAK(rc);

// 		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
// 		SUCCESS_OR_BREAK(rc);

// 		r = 0;
// 	}

// 	rc = mqtt_disconnect(&client_ctx);
// 	PRINT_RESULT("mqtt_disconnect", rc);

// 	LOG_INF("Bye!");

// 	return r;
// }

// static int start_app(void)
// {
// 	int r = 0, i = 0;

// 	while (!CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS ||
// 	       i++ < CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS) {

//     if (mess_available()) {
//       r = publisher();
//     }
		

// 		if (!CONFIG_NET_SAMPLE_APP_MAX_CONNECTIONS) {
// 			k_sleep(K_MSEC(5000));
// 		}
// 	}

// 	return r;
// }

// void add_message(const char *msg) {
//     if (message_count < MAX_MESSAGES) {
//         strncpy(influxdb_messages[message_count], msg, MAX_MESSAGE_LENGTH - 1);
//         influxdb_messages[message_count][MAX_MESSAGE_LENGTH - 1] = '\0'; // Ensure null-termination
//         message_count++;
//     } else {
//         printk("No space in influxdb_messages\n");
//     }
// }

// bool mess_available() {
//     return message_count > 0;
// }

// static char *get_mqtt_message(void)
// {
//     if (message_count > 0) {
//         return influxdb_messages[message_count - 1];
//     } else {
//         return NULL;
//     }
// }

static void print_cellular_info(void)
{
	int rc;
	int16_t rssi;
	char buffer[64];

	rc = cellular_get_signal(modem, CELLULAR_SIGNAL_RSSI, &rssi);
	if (!rc) {
		printk("RSSI %d\n", rssi);
	}

	rc = cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_IMEI, &buffer[0], sizeof(buffer));
	if (!rc) {
		printk("IMEI: %s\n", buffer);
	}
	rc = cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_MODEL_ID, &buffer[0],
				     sizeof(buffer));
	if (!rc) {
		printk("MODEL_ID: %s\n", buffer);
	}
	rc = cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_MANUFACTURER, &buffer[0],
				     sizeof(buffer));
	if (!rc) {
		printk("MANUFACTURER: %s\n", buffer);
	}
	rc = cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_SIM_IMSI, &buffer[0],
				     sizeof(buffer));
	if (!rc) {
		printk("SIM_IMSI: %s\n", buffer);
	}
	rc = cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_SIM_ICCID, &buffer[0],
				     sizeof(buffer));
	if (!rc) {
		printk("SIM_ICCID: %s\n", buffer);
	}
	rc = cellular_get_modem_info(modem, CELLULAR_MODEM_INFO_FW_VERSION, &buffer[0],
				     sizeof(buffer));
	if (!rc) {
		printk("FW_VERSION: %s\n", buffer);
	}
}

void unix_to_datetime(time_t unix_time, char *buffer, size_t buffer_size) {
    struct tm *tm_info;

    // Convert Unix time to tm structure
    tm_info = gmtime(&unix_time);

    // Format the tm structure into a human-readable string
    strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%S", tm_info);
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf)
{ 
  uint8_t * pData=(uint8_t*) buf->data;
  makeen_data_ADC *mdata=(makeen_data_ADC*) pData;
  
  mdata->batteryLvl = mdata->batteryLvl>>8 | mdata->batteryLvl<<8; // flip bytes in batteryLvl
  if (mdata->companyID == 0x017F && mdata->config[1] == 0x03)
  {
    mdata->addr = *addr;
    mdata->rssi = rssi;
    makeen_data_ADC * empty_spot_pointer = spsc_acquire(&spscBuf1);
    if (empty_spot_pointer != NULL)
    {
      memcpy(empty_spot_pointer, mdata, sizeof(makeen_data_ADC));
      spsc_produce(&spscBuf1);      
    } else {
      printk("No space in spscBuf1\n");
    } 
  }
}


void init_task(void)
{
  k_sleep(K_MSEC(1000));
  printk("Init task started\n");
  while (1)
  {
    while (!bt_is_ready())
    {
      printk("Starting Scanner\n");
      err = bt_enable(NULL);
      if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
      }
      err = bt_le_scan_start(&scan_param, scan_cb);
      if (err) {
        printk("Bluetooth scan_start failed (err %d)\n", err);
      }
    } 
    while (sntp_simple(SNTP_SERVER, 5000, &ts))
    {
      //Connect to LTE
      pm_device_action_run(modem, PM_DEVICE_ACTION_RESUME); //Turn on modem

      struct net_if *const iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
      int ret;

      // printk("Powering on modem\n");
    

      printk("Bring up network interface\n");
      ret = net_if_up(iface);
      if (ret) {
        printk("Failed to bring up network interface\n");
        return;
      }

      printk("Waiting for L4 connected\n");
      ret = net_mgmt_event_wait_on_iface(iface, NET_EVENT_L4_CONNECTED, NULL, NULL, NULL,
                K_SECONDS(120));

      if (ret) {
        printk("L4 was not connected in time\n");
        return;
      }

      printk("Waiting for DNS server added\n");
      ret = net_mgmt_event_wait_on_iface(iface, NET_EVENT_DNS_SERVER_ADD, NULL, NULL, NULL,
                K_SECONDS(20));
      if (ret) {
        printk("DNS server was not added in time\n");
        return;
      }

      printk("Retrieving cellular info\n");
      print_cellular_info();

      sntp_server.sin_family = AF_INET;
      sntp_server.sin_port = htons(123);
      ret = net_addr_pton(AF_INET, SNTP_SERVER, &sntp_server.sin_addr);
      if (ret) {
        printk("Invalid address: %d\n", ret);
        printk("\n");
        return;
      }
    }

    // int rc;
    // rc = tls_init();
    // if (rc != 0) {
    //   printk("Failed to initialize TLS. err: %d\n", rc);
    //   return;
    // } else {
    //   printk("TLS initialized\n");
    // }
    if (bt_is_ready()) {
      printk("Bluetooth initialized - killing thread\n");
      k_thread_abort(k_current_get());
    }
  }
}


void timestamp_task(void)
{
  printk("Timestamping task started\n");
   
  k_sleep(K_MSEC(1000));

  while (1)
  {
    if (!sntp_simple(SNTP_SERVER, 5000, &ts) && spsc_consumable(&spscBuf1))
    {
      makeen_data_ADC * dataptr = spsc_consume(&spscBuf1);
      if (dataptr != NULL)
      {
        makeen_data_filtered * empty_spot_pointer = spsc_acquire(&spscBuf2);
        if (empty_spot_pointer != NULL)
        {
          empty_spot_pointer->seqID = dataptr->seqID;
          empty_spot_pointer->runtimeCounter = dataptr->counter;
          empty_spot_pointer->batteryLvl = dataptr->batteryLvl;
          empty_spot_pointer->adc_val1 = dataptr->adc_val1;
          empty_spot_pointer->adc_val2 = dataptr->adc_val2;
          empty_spot_pointer->addr = dataptr->addr;
          empty_spot_pointer->rssi = dataptr->rssi;
          empty_spot_pointer->timestamp = ts.seconds;
          spsc_release(&spscBuf1);
          spsc_produce(&spscBuf2);      
        } else {
          printk("No space in spscBuf2\n");
        } 
      }
    } else {
      k_sleep(K_MSEC(250));
    }
  }
}


void decipher_task(void)
{
  printk("Decipher task started\n");

  k_sleep(K_MSEC(1000));

  while (1) {
    if (spsc_consumable(&spscBuf2) > 0)
    {
      makeen_data_filtered * dataptr = spsc_consume(&spscBuf2);
      if (dataptr != NULL)
      {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&dataptr->addr, addr_str, sizeof(addr_str));
        printk("Addr: %s\n", addr_str);
        printk("SeqID: %d\n", dataptr->seqID);
        printk("RuntimeCounter: %d\n", dataptr->runtimeCounter);
        printk("BatteryLvl: %d\n", dataptr->batteryLvl);
        printk("ADC1: %d\n", dataptr->adc_val1);
        printk("ADC2: %d\n", dataptr->adc_val2);
        printk("RSSI: %d\n", dataptr->rssi);
        printk("Timestamp: %lld\n", dataptr->timestamp);

        //Prepare InfluxDB line protocol
        // char influx_line[256];
        // snprintf(influx_line, sizeof(influx_line), "makeen_PRV_measurement,addr=%s seqID=%du,runtimeCounter=%du,batteryLvl=%du,adc_val1=%du,adc_val2=%du,rssi=%di %lld\n", addr_str, dataptr->seqID, dataptr->runtimeCounter, dataptr->batteryLvl, dataptr->adc_val1, dataptr->adc_val2, dataptr->rssi, dataptr->timestamp*1000000000);
        // printk("InfluxDB line: %s\n", influx_line);
        // add_message(influx_line);
        spsc_release(&spscBuf2);
      }
    } else {
      k_sleep(K_MSEC(250));
      // printk("No data in spscBuf2\n");
    }
  }
}


void MQTT_task(void)
{
  printk("MQTT task started\n");
  while (1)
  {
    if (mess_available()) {
      int r = -10;
      r = publisher();
      if (r == 0) {
        printk("Data sent to MQTT\n");
      } else {
        printk("Failed to send data to MQTT, errco: %d\n", r);
      }
    } else {
      k_sleep(K_MSEC(250));
    }
  }
}

#define STACKSIZE 1024
#define PRIORITY 14
K_THREAD_DEFINE(init_task_id, STACKSIZE*2, init_task, NULL, NULL, NULL, K_PRIO_PREEMPT(PRIORITY-1), 0, 0);
K_THREAD_DEFINE(timestamp_task_id, STACKSIZE, timestamp_task, NULL, NULL, NULL, K_PRIO_PREEMPT(PRIORITY), 0, 0);
K_THREAD_DEFINE(decipher_task_id, STACKSIZE, decipher_task, NULL, NULL, NULL, K_PRIO_PREEMPT(PRIORITY), 0, 0);
// K_THREAD_DEFINE(MQTT_task_id, STACKSIZE/2, MQTT_task, NULL, NULL, NULL, K_PRIO_PREEMPT(PRIORITY), 0, 0);