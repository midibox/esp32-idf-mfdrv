/*
 * MIDIbox Console
 *
 * =============================================================================
 *
 * MIT License
 *
 * Copyright (c) 2020 Thorsten Klose (tk@midibox.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * =============================================================================
 */

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "driver/uart.h"

#include "console.h"

#include "wifi.h"
#include "applemidi.h"
#include "if/lwip/applemidi_if.h"
#include "blemidi.h"
#include "uartmidi.h"


static void register_console_commands(void);


////////////////////////////////////////////////////////////////////////////////////////////////////
// Initializes the Console
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t console_init(void)
{
  /* Disable buffering on stdin */
  setvbuf(stdin, NULL, _IONBF, 0);

  /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
  esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  /* Move the caret to the beginning of the next line on '\n' */
  esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  /* Configure UART. Note that REF_TICK is used so that the baud rate remains
   * correct while APB frequency is changing in light sleep mode.
   */
  const uart_config_t uart_config = {
          .baud_rate = 115200,
          .data_bits = UART_DATA_8_BITS,
          .parity = UART_PARITY_DISABLE,
          .stop_bits = UART_STOP_BITS_1,
          .use_ref_tick = true
  };
  ESP_ERROR_CHECK( uart_param_config(CONFIG_CONSOLE_UART_NUM, &uart_config) );

  /* Install UART driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK( uart_driver_install(CONFIG_CONSOLE_UART_NUM,
          256, 0, 0, NULL, 0) );

  /* Tell VFS to use UART driver */
  esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);

  /* Initialize the console */
  esp_console_config_t console_config = {
          .max_cmdline_args = 8,
          .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
          .hint_color = atoi(LOG_COLOR_CYAN)
#endif
  };
  ESP_ERROR_CHECK( esp_console_init(&console_config) );

  /* Configure linenoise line completion library */
  /* Enable multiline editing. If not set, long commands will scroll within
   * single line.
   */
  linenoiseSetMultiLine(1);

  /* Tell linenoise where to get command completions and hints */
  linenoiseSetCompletionCallback(&esp_console_get_completion);
  linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

  /* Set command history size */
  linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
  /* Load command history from filesystem */
  linenoiseHistoryLoad(HISTORY_PATH);
#endif

  esp_console_register_help_command();
  wifi_register_console_commands();
#if APPLEMIDI_IF_ENABLE_CONSOLE
  applemidi_if_register_console_commands();
#endif
#if BLEMIDI_ENABLE_CONSOLE
  blemidi_register_console_commands();
#endif
#if UARTMIDI_ENABLE_CONSOLE
  uartmidi_register_console_commands();
#endif
  register_console_commands();

  /* Figure out if the terminal supports escape sequences */
  int probe_status = linenoiseProbe();
  if (probe_status) { /* zero indicates success */
      printf("\n"
             "Your terminal application does not support escape sequences.\n"
             "Line editing and history features are disabled.\n"
             "On Windows, try using Putty instead.\n");
      linenoiseSetDumbMode(1);
  }

  return 0; // no error
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Should be called periodically to handle console commands
////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t console_tick(void)
{
#if CONFIG_LOG_COLORS
  /* Since the terminal doesn't support escape sequences,
   * don't use color codes in the prompt.
   */
  const char* prompt = APPLEMIDI_MY_DEFAULT_NAME "> ";
#else
  const char* prompt = LOG_COLOR_I APPLEMIDI_MY_DEFAULT_NAME "> " LOG_RESET_COLOR;
#endif //CONFIG_LOG_COLORS

  /* Get a line using linenoise.
   * The line is returned when ENTER is pressed.
   */
  char* line = linenoise(prompt);
  if (line == NULL) { /* Ignore empty lines */
    return 0;
  }
  /* Add the command to the history */
  linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
  /* Save command history to filesystem */
  linenoiseHistorySave(HISTORY_PATH);
#endif

  /* Try to run the command */
  int ret;
  esp_err_t err = esp_console_run(line, &ret);
  if (err == ESP_ERR_NOT_FOUND) {
    printf("Unrecognized command\n");
  } else if (err == ESP_ERR_INVALID_ARG) {
    // command was empty
  } else if (err == ESP_OK && ret != ESP_OK) {
    printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
  } else if (err != ESP_OK) {
    printf("Internal error: %s\n", esp_err_to_name(err));
  }

  /* linenoise allocates line buffer on the heap, so need to free it */
  linenoiseFree(line);

  return 0; // no error
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Console Commands
////////////////////////////////////////////////////////////////////////////////////////////////////

static struct {
  struct arg_str *on_off;
  struct arg_end *end;
} midibox_debug_args;

static int cmd_midibox_debug(int argc, char **argv)
{
  int nerrors = arg_parse(argc, argv, (void **)&midibox_debug_args);
  if( nerrors != 0 ) {
      arg_print_errors(stderr, midibox_debug_args.end, argv[0]);
      return 1;
  }

  if( strcasecmp(midibox_debug_args.on_off->sval[0], "on") == 0 ) {
    printf("Enabled debug messages\n");
    esp_log_level_set("MIDIbox", ESP_LOG_INFO);
  } else {
    printf("Disabled debug messages - they can be re-enabled with 'midibox_debug on'\n");
    esp_log_level_set("MIDIbox", ESP_LOG_WARN); // TODO: make it configurable via console
  }

  return 0; // no error
}

static void register_console_commands(void)
{
  {
    midibox_debug_args.on_off = arg_str1(NULL, NULL, "<on/off>", "Enables/Disables debug messages");
    midibox_debug_args.end = arg_end(20);

    const esp_console_cmd_t midibox_debug_cmd = {
      .command = "midibox_debug",
      .help = "Enables/Disables MIDIbox Debug Messages",
      .hint = NULL,
      .func = &cmd_midibox_debug,
      .argtable = &midibox_debug_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&midibox_debug_cmd) );
  }
}
