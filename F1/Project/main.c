#include "ymq2.h"

int main(void)
{
	OUT(PC,13);
	IN(PA,0);
	while(1)
	{
		PCout(13)=PAin(0);
	}
}
