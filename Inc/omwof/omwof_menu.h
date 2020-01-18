/*
 * menu.h
 *
 *  Created on: 2 Dec 2019
 *      Author: oli
 */

#ifndef OMWOF_OMWOF_MENU_H_
#define OMWOF_OMWOF_MENU_H_


#include <stdint.h>
#include <stdlib.h>
#include <arm_math.h>

typedef enum menu_type {
	WEIGHT_FOLDER, WINDOW_FOLDER ,POWER_FOLDER
} enum_menu_type;

typedef union func_union{
	float32_t (*func_weight)(float32_t *, uint32_t,float32_t *);
	void (*func_window)(float32_t *, uint32_t);
	void (*func_power)(float32_t *,float32_t *, uint32_t);
}typedef_func_union;


typedef struct callback {
	typedef_func_union callback_ptr;
	char *  callback_name;
	void * next_ptr;
}callback_typedef;



typedef struct user_menu
{
	char * folder_name;
	callback_typedef * active_callback;



}menu_typedef;

extern menu_typedef *  toplevel_menu[3];

void add_new_menu(callback_typedef * menu_head,uint8_t array_len,const char * Title,const uint8_t index, enum_menu_type param_type_ID );

menu_typedef * add_menu(const char * menu_title,uint8_t index);


uint8_t add_callback(menu_typedef * menu, callback_typedef * callback );

void next_callback(menu_typedef * current_menu);

static void MX_I2C2_Init(void);
static void MX_GPIO_Init(void);



//void menu_add_callback((void*)func_ptr);




#endif /* OMWOF_OMWOF_MENU_H_ */
