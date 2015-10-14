/*
 * AdditionalTools.cpp
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#include <AdditionalTools.h>
#include <stdio.h>

float buffer[10][10];

void AdditionalTools::setBuffer(int index, float* buf, int length){
	for(int i = 0; i < length; i++){
		buffer[index][i] = buf[i];
	}
}

float* AdditionalTools::getBuffer(int index){
	return buffer[index];
}

void AdditionalTools::printfBuffer(int index, int length){
	for(int i = 0; i < length; i++){
		printf("%g", buffer[index][i]);
		if(i == length - 1){
			printf("\n");
		}
		else{
			printf(",");
		}
	}
}

