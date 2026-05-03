#include <libgen.h>
#include <string.h>


char *basename(char *filename)
{
	char *p = strrchr(filename, '/');
	return p ? p + 1 : (char *)filename;
}
