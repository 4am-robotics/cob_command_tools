#ifndef BUTTONS_H
#define BUTTONS_H

//für roscpp
#include <ros/ros.h>
#include <string>
#include <map>
#include <list>

namespace rviz 
{
  typedef std::list<button> buttonlist;
  typedef std::list<std::string> stringlist;

  class Buttons
  {
  public:
	Buttons();
       ~Buttons();	
	
	bool CreateControlPanel(); 
	Button CreateButton(string button_name, string component_name, string parameter_name);
	map<string, int> SortDict(map<string, int> dictionary);
	list<Button> uniqify_list(const list<Button> seq, int idfun);

  private:
	list <vsizers> panels;
	buttonlist stop_buttons;
	buttonlist init_buttons;
	buttonlist recover_buttons;

  }; //end of Buttons



//template for .yaml handling 
template<class T, class A>
void GetGroup_name(const std::map<T,A>& dict, int i);
void GetComponent_name(const std::map<T,A>& dict, int i);
void GetButton_name(const std::map<T,A>& dict, int i);
void GetButton_func_name(const std::map<T,A>& dict, int i);
void GetButton_param_name(const std::map<T,A>& dict, int i);

}


#endif
