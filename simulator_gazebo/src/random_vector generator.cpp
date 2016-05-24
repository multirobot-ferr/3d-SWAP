/*
 * Change Position of Static Object
 * Author: Luis Manuel Ramirez de la Cova
 * Date: March 2016
 * Organization: University of Seville, GRVC
 * Description: Algorithm that can change randomly the positions
 * of static object for the Testbed MBZIRC
 */
 
#include <fstream> 
#include <iostream>
#include <algorithm>
#include <vector>
#include <ctime>
#include <cstdlib>

#include "ros/ros.h"


//Random Generator Function
int myrandom (int i)
{
	return std::rand()%i;
}

int main()
{
	std::srand (unsigned(std::time(0)));
	std::vector <int> myvector;
	
	//Set some values
	for(int i=1; i<10; ++i) myvector.push_back(i);
	
	//Using built-in random generator
	std::random_shuffle (myvector.begin(), myvector.end());
	
	//Using myrandom
	std::random_shuffle (myvector.begin(), myvector.end(), myrandom);
	
	//Print out content
	std::cout << "myvector contains:";
	for(std::vector<int>::iterator it=myvector.begin(); it!=myvector.end(); ++i)
	{
		std::cout << ' ' << *it;
	}
	
	std::cout << '\n';
	
	return 0;
}

