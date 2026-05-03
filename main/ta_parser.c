#include "stdio.h"
#include "strings.h"
#include "string.h"
#include "esp_linenoise.h"
#include "wifi_manager.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota_updater.h"

static esp_linenoise_handle_t handle;

/*
 There's multiple synonyms for the words people can enter here. For example
 'press' and 'push' can be handled as one and the same word. In order to simplify
 this, we first map a sentence to a set of token, where a token represents
 a set of similar words. For instance, 'press' and 'push' both get token W_PRESS.
*/

typedef struct {
	const char *word;
	int token;
} word_token_t;

//These are the tokens.
#define W_WEND 0 //end of words
#define W_WITH 1
#define W_IN 2
#define W_ON 3
#define W_UNDER 4
#define W_QUIT 5
#define W_SAVE 6
#define W_LOOK 7
#define W_AT 8
#define W_ROOM 9
#define W_SIT 10
#define W_CHAIR 11
#define W_LIFT 12
#define W_GO 13
#define W_AROUND 14
#define W_PRESS 15
#define W_BUTTONS 16
#define W_OPEN 17
#define W_SERVER 18
#define W_HIT 19
#define W_CONTROL 20
#define W_PANEL 21
#define W_FOTA 22
#define W_UPDATE 23
#define W_FIRMWARE 24
#define W_WIFI 25
#define W_SETTINGS 26
#define W_FILLER 1000 //word will get ignored

//Here's the mapping of a word to the token that represents that word.
static const word_token_t word_token[]={
	{"with", W_WITH},
	{"using", W_WITH},
	{"through", W_WITH},
	{"thru", W_WITH},
	{"in", W_IN},
	{"inside", W_IN},
	{"into", W_IN},
	{"on", W_ON},
	{"onto", W_ON},
	{"under", W_UNDER},
	{"underneath", W_UNDER},
	{"beneath", W_UNDER},
	{"below", W_UNDER},
	{"quit", W_QUIT},
	{"exit", W_QUIT},
	{"finish", W_QUIT},
	{"save", W_SAVE},
	{"restore", W_SAVE},
	{"look", W_LOOK},
	{"glance", W_LOOK},
	{"examine", W_LOOK},
	{"inspect", W_LOOK},
	{"ls", W_LOOK},
	{"at", W_AT},
	{"upon", W_AT},
	{"room", W_ROOM},
	{"sit", W_SIT},
	{"chair", W_CHAIR},
	{"enter", W_GO},
	{"go", W_GO},
	{"elevator", W_LIFT},
	{"lift", W_LIFT},
	{"around", W_AROUND},
	{"press", W_PRESS},
	{"push", W_PRESS},
	{"button", W_BUTTONS},
	{"buttons", W_BUTTONS},
	{"open", W_OPEN},
	{"unlock", W_OPEN},
	{"computer", W_SERVER},
	{"server", W_SERVER},
	{"hit", W_HIT},
	{"strike", W_HIT},
	{"kick", W_HIT},
	{"control", W_CONTROL},
	{"panel", W_PANEL},
	{"fota", W_FOTA},
	{"update", W_UPDATE},
	{"firmware", W_FIRMWARE},
	{"wifi", W_WIFI},
	{"settings", W_SETTINGS},
	{"a", W_FILLER},
	{"the", W_FILLER},
	{"that", W_FILLER},
	{"this", W_FILLER},
	{NULL, 0}
};

typedef void (action_cb_t)(const int *words, const char *prompt);

/*
 Now we have the tokens, we can now map sets of tokens to actions. One set of tokens
 (e.g. W_LOOK, W_AT, W_LIFT can have either a description (which is printed when that
 set of tokens is entered) or an action (which is a function that will be executed
 when a set of token is entered). If a set of token has none of these two defined, the
 parser will go down the list of actions until it finds an entry that does have one
 of those defined.
*/
typedef struct {
	int words[8];
	const char *response;
	action_cb_t *cb;
} action_t;


/*
 These defines abstract that behaviour. DESCRIBE(WL(token, token,...), "text") will
 print a description when those tokens are entered. ACTION(WL(token, token, ...), callback)
 will execute the callback function when those tokens are entered. Each of those two
 can be preceeded with zero or more SYNONYM(WL(token, token, ...)) statements that
 are effectively aliases.
*/
#define SYNONYM(a) {a, NULL, NULL},
#define DESCRIBE(a, b) {a, b, NULL},
#define ACTION(a, b) {a, NULL, b},
#define WL(a...) {a}

static action_cb_t action_elevator;
static action_cb_t action_control_panel;

static const action_t actions[]={
	SYNONYM(WL(W_LOOK))
	SYNONYM(WL(W_LOOK, W_AROUND))
	SYNONYM(WL(W_LOOK, W_AT, W_ROOM))
	DESCRIBE(WL(W_LOOK, W_ROOM), "You are in a nondescript, window-less room. In a corner is a chair. In the other corner " \
							 "is a boxy device with lights on it that says \"server\". Behind you is what looks like an " \
							 "elevator, its doors are open. Just outside the broken fourth wall, there is a rectangular " \
							 "table covered in knobs, dials, and levers that says \"control panel\" (use this to update " \
							 "your badge firmware).")
	SYNONYM(WL(W_LOOK, W_CHAIR))
	DESCRIBE(WL(W_LOOK, W_AT, W_CHAIR), "The chair is a simple wooden kitchen chair. It looks sturdy enough to sit down on.")
	SYNONYM(WL(W_SIT))
	SYNONYM(WL(W_SIT, W_CHAIR))
	DESCRIBE(WL(W_SIT, W_ON, W_CHAIR), "You sit down on the chair. You immediately stand up again; the wood is hard and not very comfortable.")
	SYNONYM(WL(W_LOOK, W_LIFT))
	DESCRIBE(WL(W_LOOK, W_AT, W_LIFT), "You see two large, open metal elevator doors that open up to a well-lit elevator. Inside the elevator is a control panel with three buttons.")
	SYNONYM(WL(W_LIFT))
	SYNONYM(WL(W_GO, W_LIFT))
	ACTION(WL(W_GO, W_IN, W_LIFT), action_elevator)
	SYNONYM(WL(W_LOOK, W_AT, W_BUTTONS))
	DESCRIBE(WL(W_LOOK, W_BUTTONS), "You can make out that they're labeled with the numbers '1', '2' and '3' respectively.")
	DESCRIBE(WL(W_PRESS, W_BUTTONS), "You cannot press the elevator buttons from here.")
	DESCRIBE(WL(W_OPEN, W_LIFT), "The lift doors are already open.")
	DESCRIBE(WL(W_OPEN, W_SERVER), "You don't have the right screwdriver for that.")
	DESCRIBE(WL(W_OPEN, W_CHAIR), "It doesn't open!")
	SYNONYM(WL(W_LOOK, W_SERVER))
	DESCRIBE(WL(W_LOOK, W_AT, W_SERVER), "It does not work. Seems someone should program something to run on it.")
	DESCRIBE(WL(W_HIT, W_SERVER), "You'd rather not; that cannot be good for its hard disk.")
	DESCRIBE(WL(W_HIT, W_CHAIR), "The chair is unimpressed.")
	DESCRIBE(WL(W_HIT, W_LIFT), "You fail to even leave a dent.")
	SYNONYM(WL(W_LOOK, W_CONTROL, W_PANEL))
	DESCRIBE(WL(W_LOOK, W_AT, W_CONTROL, W_PANEL), "Just outside the broken fourth wall, there is a " \
								"table covered in knobs, dials, and levers that says \"control " \
								"panel\" (use this to update your badge firmware).")
	SYNONYM(WL(W_FOTA))
	SYNONYM(WL(W_WIFI))
	SYNONYM(WL(W_SETTINGS))
	SYNONYM(WL(W_CONTROL, W_PANEL))
	SYNONYM(WL(W_UPDATE, W_FIRMWARE))
	ACTION(WL(W_UPDATE), action_control_panel)
	{{0}, NULL, NULL}
};

//The frotz main function is called 'main'. We leave it like this as we don't want
//to touch the frotz code too much and main() otherwise is unused.
int main(int argc, char **argv);

//This starts Frotz with the indicated episode of Zork
static void start_zork(int which) {
	//Command line arguments to Zork
	char *argv_f[]={
		"frotz", "-f", "ansi", NULL
	};
	//Zork expects the command line arguments to be writable, so copy them
	//to writable memory.
	char **argv=calloc(100, sizeof(char*));
	int n=0;
	for (int i=0; argv_f[i]; i++) {
		argv[i]=strdup(argv_f[i]);
		n=i+1;
	}
	//Add the path to the game the user wants to play
	argv[n]=malloc(100);
	sprintf(argv[n], "/spiffs/ZORK%d.DAT", which);
	n++;
	//Invoke Frotz
	main(n, argv);
	//Clean up
	for (int i=0; i<n; i++) free(argv[i]);
	free(argv);
}

//Action callback to start zork.
static void action_elevator(const int *words, const char *prompt) {
	printf("You enter the elevator. What button do you want to press? Enter '1', '2' or '3'.\n");
	char buf[12];
	esp_linenoise_get_line(handle, buf, 10);
	int n=atoi(buf);
	if (n==1 || n==2 || n==3) {
		start_zork(n);
		printf("Phew, what a ride!\n");
	}
	printf("You exit the elevator again\n");
}

//Maps a word into its corresponding token. Returns the token or W_WEND if nothing
//matches.
static int lookup_word(const char *word) {
	for (int i=0; word_token[i].word!=NULL; i++) {
		if (strcasecmp(word_token[i].word, word)==0) {
			return word_token[i].token;
		}
	}
	return W_WEND;
}

//Return true if the character can be part of a word.
static int valid_char(char p) {
	const char *valid="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-";
	for (int i=0; valid[i]!=0; i++) {
		if (valid[i]==p) return 1;
	}
	return 0;
}

#define COLUMNS 72
#define MAX_CROP 20

//This function takes a long descriptor text and wraps it into COLUMNS columns,
//attempting not to split words.
static void format_limit_col(const char *str) {
	int newline=0;
	int last_space=0;
	for (int i=0; i<COLUMNS; i++) {
		if (str[i]==0) {
			//End of string. All done.
			printf("%s\n", str);
			return;
		}
		if (str[i]=='\n') {
			newline=i;
			break;
		}
		if (str[i]==' ' || str[i]=='-') {
			last_space=i;
		}
	}
	//line to output
	char line[COLUMNS+1]={0};
	int res=0;
	if (newline) {
		//copy everything up to the newline
		strncpy(line, str, newline);
		res=newline+1;
	} else {
		//copy everything up to the last space before reaching
		//COLUMNS.
		strncpy(line, str, last_space);
		res=last_space+1;
	}
	printf("%s\n", line);
	//Recursively parse the remainder of the text
	format_limit_col(str+res);
}

static void action_control_panel(const int *words, const char *prompt) {
	format_limit_col(
		"You approach the control panel. A flickering screen reads:\n"
		"\n"
		"=== BADGE CONTROL PANEL ===\n"
		"\n"
		"From here you can perform an over-the-air firmware update "
		"by connecting your badge to a WiFi network.\n"
		"\n"
		"IMPORTANT: The network must NOT have a captive portal "
		"(so you cannot use Marina Bay Sands network, for instance). "
		"Use a mobile hotspot or another network without a login page.\n"
		"\n"
		"Type 'scan' to scan for WiFi networks, 'other' to enter an "
		"SSID manually, or 'exit' to return."
	);

	char buf[256];
	char ssid[33];
	char password[65];
	wifi_auth_mode_t authmode = WIFI_AUTH_OPEN;

	while (1) {
		esp_linenoise_get_line(handle, buf, sizeof(buf) - 2);

		if (strcasecmp(buf, "exit") == 0) {
			printf("You step away from the control panel.\n");
			return;
		}

		if (strcasecmp(buf, "scan") == 0) {
			printf("Scanning for WiFi networks...\n");
			wifi_ap_record_t *aps = NULL;
			size_t n = wifi_scan_aps(&aps);

			if (n == 0) {
				printf("No networks found. Try 'scan' again, "
				       "'other' to enter an SSID manually, "
				       "or 'exit' to return.\n");
				continue;
			}

			printf("\nAvailable networks:\n");
			for (size_t i = 0; i < n; i++) {
				const char *sec = (aps[i].authmode == WIFI_AUTH_OPEN)
				                  ? "Open" : "Secured";
				printf("  %2d) %-32s  RSSI: %d  [%s]\n",
				       (int)(i + 1), (char *)aps[i].ssid,
				       aps[i].rssi, sec);
			}
			printf("\nType the number of the network you want to "
			       "connect to, or 'exit' to return.\n");

			esp_linenoise_get_line(handle, buf, sizeof(buf) - 2);

			if (strcasecmp(buf, "exit") == 0) {
				free(aps);
				printf("You step away from the control panel.\n");
				return;
			}

			int choice = atoi(buf);
			if (choice < 1 || choice > (int)n) {
				printf("Invalid choice.\n");
				free(aps);
				continue;
			}

			memset(ssid, 0, sizeof(ssid));
			strncpy(ssid, (char *)aps[choice - 1].ssid, 32);
			authmode = aps[choice - 1].authmode;
			free(aps);

			memset(password, 0, sizeof(password));
			if (authmode != WIFI_AUTH_OPEN) {
				printf("Enter password for '%s': ", ssid);
				esp_linenoise_get_line(handle, password,
				                       sizeof(password) - 1);
			}
		} else if (strcasecmp(buf, "other") == 0) {
			printf("Enter the SSID: ");
			memset(ssid, 0, sizeof(ssid));
			esp_linenoise_get_line(handle, ssid, sizeof(ssid) - 1);

			printf("Enter password (leave empty for open network): ");
			memset(password, 0, sizeof(password));
			esp_linenoise_get_line(handle, password,
			                       sizeof(password) - 1);

			authmode = (strlen(password) > 0)
			           ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
		} else {
			printf("Type 'scan', 'other', or 'exit'.\n");
			continue;
		}

		printf("Connecting to '%s'...\n", ssid);
		wifi_connect(ssid, password, authmode, 5);

		int connected = 0;
		for (int step = 0; step < 60; step++) {
			vTaskDelay(pdMS_TO_TICKS(500));
			esp_netif_t *netif =
				esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
			if (!netif) continue;
			esp_netif_ip_info_t ipi;
			if (esp_netif_get_ip_info(netif, &ipi) == ESP_OK
			    && ipi.ip.addr != 0) {
				printf("\nConnected to '%s'!\n", ssid);
				printf("IP address: " IPSTR "\n", IP2STR(&ipi.ip));
				connected = 1;
				break;
			}
		}

		if (!connected) {
			printf("Failed to connect to '%s' within 30 seconds.\n",
			       ssid);
			printf("Type 'scan' to try again, 'other' to enter an "
			       "SSID manually, or 'exit' to return.\n");
			continue;
		}

		printf("\nType 'disconnect'/'exit' to disconnect and "
		       "return, or 'fota'/'update'/'continue' to "
		       "perform the firmware update.\n");

		while (1) {
			esp_linenoise_get_line(handle, buf, sizeof(buf) - 2);

			if (strcasecmp(buf, "disconnect") == 0
			    || strcasecmp(buf, "exit") == 0) {
				printf("Disconnecting from WiFi...\n");
				wifi_disconnect();
				vTaskDelay(pdMS_TO_TICKS(500));
				printf("Disconnected. You step away from the "
				       "control panel.\n");
				return;
			}

			if (strcasecmp(buf, "fota") == 0
			    || strcasecmp(buf, "update") == 0
			    || strcasecmp(buf, "continue") == 0) {
				
				// Perform OTA update
				ota_update();
				printf("\nYou step away from the control "
				       "panel.\n");
				return;
			}

			printf("Type 'disconnect', 'exit', 'fota', 'update', "
			       "or 'continue'.\n");
		}
	}
}

//Parse ASCII sentence and generates a response. Returns 1 on success, or anything
//else on failure (e.g. failure to tokenize, no action matching the words specified)
static int parse_sentence(const char *sentence) {
	char word[64]={0};
	int tokens[8]={0};
	int wp=-1;
	int no_tokens=0;
	const char *p=sentence;
	//Parse the sentence into tokens. After this, tokens[0..no_tokens] will contain
	//the tokens the sentence consisted of.
	while (1) {
		if (valid_char(*p) && (wp<62)) {
			//Found one more char of the current word.
			wp++;
			word[wp]=*p;
		} else {
			if (wp>=0) {
				//End of the word. Look up the token it represents.
				word[wp+1]=0;
				int token=lookup_word(word);
				if (token==W_WEND) {
					printf("Sorry, I don't know the word \"%s\".\n", word);
					return -1;
				}
				if (token!=W_FILLER) {
					//If word should not be ignored, add to token array.
					tokens[no_tokens++]=token;
					if (no_tokens==sizeof(tokens)-1) break;
				}
			}
			wp=-1;
		}
		//Note: we check for the zero terminator after the valid_char stuff in order
		//to have the zero terminator trigger a word end.
		if (*p==0) break;
		p++;
	}
	tokens[no_tokens]=W_WEND; //terminator

	//exit if no tokens found (e.g. user just pressed return)
	if (no_tokens==0) return -1;

//	for (int i=0; i<8; i++) printf("%d ", tokens[i]);
//	printf("\n");

	//See if the tokens we parsed match any of the actions.
	int n=-1;
	for (int i=0; actions[i].words[0]!=0; i++) {
		int found=1;
		for (int j=0; j<8; j++) {
			if (actions[i].words[j]!=tokens[j]) {
				found=0;
				break;
			}
			if (tokens[j]==W_WEND) {
				//everything matches
				break;
			}
		}
		if (found) {
			n=i;
			break;
		}
	}

	if (n==-1) {
		//Words parsed into valid tokens, but no corresponding action was found.
		//(e.g. when user enters nonsense like 'lift look push')
		printf("Sorry, I don't know how to do that.\n");
		return -1;
	}

	//We might have arrived on a synonym. If so, go down the list until we find
	//an action that does have a response or callback associated.
	while (!actions[n].response && !actions[n].cb && actions[n].words[0]!=0) n++;

	//Print response or execute callback.
	if (actions[n].response) {
		format_limit_col(actions[n].response);
	}
	if (actions[n].cb) {
		actions[n].cb(tokens, sentence);
	}
	//All done.
	return 1;
}

#define INPUT_BUFFER_SIZE 512

void ta_parser_main() {
	//Configure linenoise. We use this library to get a commandline prompt.
	esp_linenoise_config_t config;
	esp_linenoise_get_instance_config_default(&config);
	config.max_cmd_line_length=INPUT_BUFFER_SIZE-2;
	esp_linenoise_create_instance(&config, &handle);

	//On startup, we tell the user where they are.
	printf("\n\n Nondescript Room\n\n");
	parse_sentence("look");

	//Now read lines and parse them.
	char buf[INPUT_BUFFER_SIZE];
	while(1) {
		esp_linenoise_get_line(handle, buf, INPUT_BUFFER_SIZE-2);
		parse_sentence(buf);
	}
}



