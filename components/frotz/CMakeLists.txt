
set(FROTZ_FILES
	"blorb/blorblib.c"
	"common/buffer.c" "common/err.c" "common/fastmem.c" "common/files.c"
	"common/getopt.c" "common/hotkey.c" "common/input.c"
	"common/main.c" "common/math.c" "common/missing.c" 
	"common/object.c" "common/process.c" "common/quetzal.c" "common/random.c"
	"common/redirect.c" "common/screen.c" "common/sound.c" "common/stream.c"
	"common/table.c" "common/text.c" "common/variable.c"
)

set (FROTZ_INCLUDE "blorb/" "common/" "dumb/")

list(TRANSFORM FROTZ_FILES PREPEND "frotz/src/")
list(TRANSFORM FROTZ_INCLUDE PREPEND "frotz/src/")

idf_component_register( SRCS "basename.c" ${FROTZ_FILES} "esp/dinit.c" "esp/dinput.c" "esp/doutput.c" "esp/dpic.c"
						PRIV_REQUIRES esp_linenoise esp_timer
                        INCLUDE_DIRS ${FROTZ_INCLUDE} ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=char-subscripts)