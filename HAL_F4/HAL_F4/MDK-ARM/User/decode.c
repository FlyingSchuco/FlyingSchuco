#include "decode.h"
int decode(char *buffer, int *output)
{
	int len = strlen(buffer);
	int i = 0;
	int delta_x = 0, delta_y = 0;
	//printf("len = %d\r\n",len);
	//printf("content = %s\r\n",buffer);
	if(len)
	{
		if(buffer[0] == 'T')
		{
			int flag = 1;
			if(buffer[1]=='-') flag = -1;
			for(i=2;buffer[i]!=' ';++i)
			{
				delta_x *= 10;
				delta_x += flag*(buffer[i]-'0');
			}
			if(buffer[++i]=='-') flag = -1;
			for(++i;buffer[i]!=' ';++i)
			{
				delta_y *= 10;
				delta_y += flag*(buffer[i]-'0');
			}
			output[0] = delta_x;
			output[1] = delta_y;
			return 1;
		}
	}
	return 0;
}
