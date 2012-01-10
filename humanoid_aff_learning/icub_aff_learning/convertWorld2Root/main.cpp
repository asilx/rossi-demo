#include "iostream"
#include "string"

int main()
{
	double x, y, z;
	std::string s1, s2;
	
	while(true)
	{
		std::cout<<"enter the location command"<<std::endl;
		std::cin>>s1>>x>>y>>z;
		std::cout
		<<-z-0.04<<" "<<-x<<" "<<y-0.5484<<std::endl;
	}
	return 0;
}
