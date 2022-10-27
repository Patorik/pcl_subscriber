#include <iostream>
#include <rosnode.h>

int main(int argc, char **argv)
{
    std::string topic_name_sub = "/os_cloud_node/points";
    RosNode node(argc, argv, topic_name_sub, 10);
    
    return 0; 
}