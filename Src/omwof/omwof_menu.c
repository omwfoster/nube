/*
 * omwof_menu.c
 *
 *  Created on: 2 Dec 2019
 *      Author: oli
 */

#include "omwof/omwof_menu.h"

menu_typedef * toplevel_menu[2];

menu_typedef * add_menu(const char * menu_title, uint8_t index) {

	menu_typedef * m = malloc(sizeof(menu_typedef));
	m->folder_name = strdup(menu_title);
	toplevel_menu[index] = m;
	toplevel_menu[index]->callback_head = NULL;
	return m;

}

uint8_t add_weight_callback(menu_typedef * menu, char * callback_name,
		typedef_func_union * t_func) {

	callback_typedef * cb = malloc(sizeof(callback_typedef));
	cb->callback_ptr->func_weight = t_func->func_weight;
	if (menu->callback_head = NULL) // create first item
			{
		menu->callback_head = cb;
		menu->folder_name = strdup(callback_name);
		menu->callback_head->next_ptr = cb; // point next item to self to  create a loop
		return 1;
	}
	callback_typedef * current_callback = menu->callback_head ;

	while (current_callback->next_ptr != cb) {
		current_callback = current_callback->next_ptr; // move to end of the list
	}

	current_callback->next_ptr = cb;
	cb->callback_name = strdup(callback_name);
	cb->callback_ptr->func_weight =  t_func->func_weight;
	cb->next_ptr = (void *)menu->callback_head;


}

uint8_t add_window_callback(menu_typedef * menu, char * callback_name,
		typedef_func_union * t_func) {

	callback_typedef * cb = malloc(sizeof(callback_typedef));
	cb->callback_ptr->func_window = t_func->func_window;
	if (menu->callback_head = NULL) // create first item
			{
		menu->callback_head = cb;
		menu->folder_name = strdup(callback_name);
		menu->callback_head->next_ptr = cb; // point next item to self to  create a loop
		return 1;
	}
	callback_typedef * current_callback = menu->callback_head ;

	while (current_callback->next_ptr != cb) {
		current_callback = current_callback->next_ptr; // move to end of the list
	}

	current_callback->next_ptr = cb;
	cb->callback_name = strdup(callback_name);
	cb->callback_ptr->func_window =  t_func->func_window;
	cb->next_ptr = (void *)menu->callback_head;


}



void delete_menu() {
}

