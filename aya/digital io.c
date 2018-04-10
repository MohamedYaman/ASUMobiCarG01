#include "uCongfig.h"
#include "STDtypes.h"

/*Global Variables*/
/*ISR Code*/


int main(void)
{
	DDRA = 0xFF;	
	
	sint8 i = 0 ;
    while(1)
    {
    for (i=0 ; i<=7 ; i++)
 {
		
	PORTA |= (1<<i);			//to make pin PA1 be on.
	_delay_ms(250);
}

	for (i=8; i>=0; i--)
	{
			
			PORTA &=~ (1<<(i-1));			//to make pin PA1 be on.
			_delay_ms(250);
	}
	}

}
	
