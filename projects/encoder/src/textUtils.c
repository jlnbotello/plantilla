#include "textUtils.h"


void uint32ToASCII(uint32_t valor, char * toASCII){
	uint8_t aux[10];
	uint8_t N=digitosDec(valor,aux);
	for(uint8_t i=0; i<N;i++)
		toASCII[i]=aux[i]+'0';
	toASCII[N]='\0';
}

uint8_t digitosDec(uint32_t valor, uint8_t * digitos){

	uint8_t cant=1;
	uint8_t aux[10];
	aux[cant-1]=valor%10;
	while(valor>=10){
		cant++;
		valor/=10;
		aux[cant-1]=valor%10;
	}
	for(uint8_t i=0;i<cant;i++){
		digitos[cant-1-i]=aux[i];
	}
	return cant;
}



