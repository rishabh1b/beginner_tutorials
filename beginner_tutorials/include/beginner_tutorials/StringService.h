/**
* @file StringService.h header file for services that can be provided
* @author Rishabh Biyani (rishabh1b)
* @copyright MIIT license (c) Rishabh Biyani 2017
*/

#include <string>

/*
* @param global variable which can be changed by a service
*/
std::string curr_pub_string = "Rishabh Biyani";

/**
* @brief method to replace the string being published
* @param a string to be replaced
*/

void replacePublishedString(std::string str);
