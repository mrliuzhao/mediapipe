
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <iostream>
 
using namespace boost::property_tree;
 
//解析Json
bool ParseJson()
{
    std::string str = "{\"ID\":100,\"Student\":[{\"Name\":\"April\"},{\"Name\":\"Harris\"}]}";
    std::stringstream stream(str);
    ptree strTree;
    try {
        read_json(stream, strTree);
    }
    catch (ptree_error & e) {
        return false;
    }
 
    try {
        int id = strTree.get<int>("ID");   
        std::cout << "ID: " <<  id << std::endl;
        ptree names = strTree.get_child("Student");  
        BOOST_FOREACH(ptree::value_type &name, names)
        {
            std::stringstream key, value;
            // write_json(key, name.first);
            write_json(value, name.second);
            // std::string keyStr = key.str();
            std::string valStr = value.str();
            // std::cout << "Key: " <<  keyStr << "; Value: " << valStr << std::endl;
            std::cout << "Value: " << valStr << std::endl;
        }
    }
    catch (ptree_error & e)
    {
        return false;
    }
    return true;
}
 
 
//构造Json
bool InsertJson()
{
    std::string str = "{\"ID\":0,\"Student\":[{\"Name\":\"April\"},{\"Name\":\"Harris\"}]}";
    std::stringstream stream(str);
    ptree strTree;
    try {
        read_json(stream, strTree);
    }
    catch (ptree_error & e) {
        return false;
    }
    ptree subject_info;
    ptree array1, array2, array3;
    array1.put("course", "Java");
    array2.put("course", "C++");
    array3.put("course", "MySql");
    subject_info.push_back(make_pair("", array1));
    subject_info.push_back(make_pair("", array2));
    subject_info.push_back(make_pair("", array3));
 
    strTree.put_child("Subject", subject_info);
    std::stringstream s;
    write_json(s, strTree);
    std::string outstr = s.str();
    std::cout << "After Insertion: " <<  outstr << std::endl;
    
    return true;
}
 
int main()
{
    // ParseJson();
    // InsertJson();

// {
// 	"command": "startTraining",
// 	"msgType": "rpc_response",
// 	"msgId": "2", //该ID值为请求报文中的msgId
// 	"msgData": {
// 		"status": 0, //状态:0:失败;1:成功 
//         "msg":"失败原因"
// 	}
// }
    
    ptree strTree;
    strTree.put("command", "startTraining");
    strTree.put("msgType", "rpc_response");
    strTree.put("msgId", "A327D96D-1AF6-4966-B0F4-B9A37AE51105");
    ptree msg_data;
    msg_data.put("status", 1);
    std::string errorMsg = "Test Error!!!";
    msg_data.put("msg", errorMsg);
    strTree.put_child("msgData", msg_data);
    std::stringstream s;
    write_json(s, strTree);
    std::string outstr = s.str();
    std::cout << "Created Json: " <<  outstr << std::endl;

    return 0;
}

