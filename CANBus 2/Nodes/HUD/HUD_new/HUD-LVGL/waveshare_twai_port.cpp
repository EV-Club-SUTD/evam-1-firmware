#include "waveshare_twai_port.h"
#include "can_data_parser.h"

// ===== Function to handle received messages =====
static void handle_rx_message(twai_message_t &message)
{
  // --- Filter: only process messages we care about ---
  if (message.identifier != 0x08 && 
      message.identifier != 0x24 && 
      message.identifier != 0x34 && 
      message.identifier != 0x35 && 
      message.identifier != 0x36 && 
      message.identifier != 0x37 && 
      message.identifier != 0x38) {
    return;  // Ignore this message, don't process it
  }

  // --- Process received message ---
  if (message.extd)
  {
    Serial.println("Message is in Extended Format");
  }
  else
  {
    Serial.println("Message is in Standard Format");
  }
  Serial.printf("ID: 0x%08X\nByte:", message.identifier);
  if (!(message.rtr))
  {
    for (int i = 0; i < message.data_length_code; i++)
    {
      Serial.printf(" %d = %02X,", i, message.data[i]);
    }
    Serial.println("");
  }
  
  // --- Parse the message and extract vehicle data ---
  can_data_parse(message);
}

// ===== Function to initialize the TWAI driver =====
bool waveshare_twai_init()
{
  // --- Initialize configuration structures using macro initializers ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_LISTEN_ONLY);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // --- Install TWAI driver ---
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    Serial.println("Failed to install driver");
    return false;
  }
  Serial.println("Driver installed");

  // --- Start TWAI driver ---
  if (twai_start() != ESP_OK)
  {
    Serial.println("Failed to start driver");
    return false;
  }
  Serial.println("Driver started");

  // --- Reconfigure alerts to detect frame receive, Bus-Off error, and RX queue full states ---
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    Serial.println("CAN Alerts reconfigured");
  }
  else
  {
    Serial.println("Failed to reconfigure alerts");
    return false;
  }
  
  // --- Initialize CAN data parser ---
  can_data_init();

  return true;
}

// ===== Function to receive messages via TWAI =====
void waveshare_twai_receive()
{
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // --- Handle alerts ---
  if (alerts_triggered & TWAI_ALERT_ERR_PASS)
  {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
  {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
  {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // --- Check if message is received ---
  if (alerts_triggered & TWAI_ALERT_RX_DATA)
  {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK)
    {
      handle_rx_message(message);
    }
  }
}