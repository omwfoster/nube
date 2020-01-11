/*
 * omwof_menu.c
 *
 *  Created on: 2 Dec 2019
 *      Author: oli
 */

#include "omwof/omwof_menu.h"

menu_typedef * toplevel_menu[3];
menu_typedef volatile * active_menu;

void add_new_menu(callback_typedef * menu_head,uint8_t array_len,char * Title, enum_menu_type param_type_ID )
{
	if(!toplevel_menu[0])
	{

	};

}

menu_typedef * add_menu(const char * menu_title, uint8_t index) {

	menu_typedef * m = malloc(sizeof(menu_typedef));
	m->folder_name = strdup(menu_title);
	toplevel_menu[index] = m;
	toplevel_menu[index]->active_callback = NULL;
	return m;

}

uint8_t add_callback(menu_typedef * menu, char * callback_name,
		typedef_func_union * t_func) {

	if (menu->active_callback == NULL) // create first item
	{
		menu->active_callback = malloc(sizeof(callback_typedef));
		menu->active_callback->callback_ptr = t_func;
		menu->active_callback->callback_name = strdup(callback_name);
		menu->active_callback->next_ptr = menu->active_callback; // point next item to self to  create a loop
		return 1;
	}

	callback_typedef * current_callback = menu->active_callback;

	while (current_callback->next_ptr != menu->active_callback->next_ptr) {
		current_callback = current_callback->next_ptr; // move to end of the list
	}

	current_callback->next_ptr = malloc(sizeof(callback_typedef));
	((callback_typedef *) current_callback->next_ptr)->callback_ptr = t_func;
	((callback_typedef *) current_callback->next_ptr)->callback_name = strdup(
			callback_name);
	((callback_typedef *) current_callback->next_ptr)->next_ptr =
			menu->active_callback;

}

void next_callback(menu_typedef * current_menu) {

	if (current_menu) {
		current_menu->active_callback = current_menu->active_callback->next_ptr;
	}
}

void delete_menu() {
}



