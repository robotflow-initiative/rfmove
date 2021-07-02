//
// Created by yongxi on 2021/5/25.
//

#ifndef MOVEIT_NO_ROS_EXAMPLECLASS_H
#define MOVEIT_NO_ROS_EXAMPLECLASS_H

#include <string>
#include <vector>

class ExampleChild {
public:
    ExampleChild(const std::string& str);
    const std::string& getName() const;
    void setName(const std::string& str);
private:
    std::string name;
};

void changeChildName(ExampleChild& child, const std::string& str);
void changeVectorContains(std::vector<int>& vec);

class ExampleClass {
public:
    ExampleClass() = default;
    void AddString(const std::string& str);
    const std::vector<std::string>& GetStringVec() const;
    void AddChild(const std::string& str);
    void AddChildPtr(const std::string& str);
    const std::vector<ExampleChild>& GetChildVec();
    const std::vector<ExampleChild*>& GetChildPtrVec();
    void copyStringVec(std::vector<std::string>&);
private:
    std::vector<std::string>stringVector;
    std::vector<ExampleChild>children;
    std::vector<ExampleChild*> childrenPtr;
};

#endif //MOVEIT_NO_ROS_EXAMPLECLASS_H
