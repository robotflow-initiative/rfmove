//
// Created by yongxi on 2021/5/25.
//
#include "robot_model/ExampleClass.h"

ExampleChild::ExampleChild(const std::string &str) {
    name = str;
}

const std::string& ExampleChild::getName() const{
    return name;
}

void changeChildName(ExampleChild& child, const std::string& str){
    child.setName(str);
}

void changeVectorContains(std::vector<int>& vec) {
    vec[0] = 99;
}

void ExampleChild::setName(const std::string& str){
    name = str;
}

void ExampleClass::AddString(const std::string& str) {
    stringVector.push_back(str);
}

void ExampleClass::AddChild(const std::string& str) {
    children.emplace_back(str);
}

void ExampleClass::AddChildPtr(const std::string& str) {
    childrenPtr.push_back(new ExampleChild(str));
}

const std::vector<ExampleChild>& ExampleClass::GetChildVec() {
    return children;
}

const std::vector<ExampleChild*>& ExampleClass::GetChildPtrVec() {
    return childrenPtr;
}

const std::vector<std::string>& ExampleClass::GetStringVec() const {
    return stringVector;
}

void ExampleClass::copyStringVec(std::vector<std::string>& container) {
    container.reserve(stringVector.size());
    for (size_t i=0; i<stringVector.size(); i++) {
        container[i] = stringVector[i];
    }
}
