set(srcs 
		"main.c" 
    "telnet.c"
		"console.c"
    "uart.c"
    "wifi_sta.c"
		"http.c"
		"mqtt.c"
		"ota.c"
    )


idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
		EMBED_FILES "index.html"
                    )

