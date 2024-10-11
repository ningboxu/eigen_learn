#include <iostream>
#include <string>

// 基类
class Animal {
public:
    virtual void makeSound() = 0;
};

// 派生类
class Dog : public Animal {
public:
    void makeSound() override {
        std::cout << "Dog barks." << std::endl;
    }
};

class Cat : public Animal {
public:
    void makeSound() override {
        std::cout << "Cat meows." << std::endl;
    }
};

// 工厂类
class AnimalFactory {
public:
    static Animal* createAnimal(const std::string& type) {
        if (type == "dog") {
            return new Dog();
        } else if (type == "cat") {
            return new Cat();
        } else {
            return nullptr;
        }
    }
};

int main() {
    Animal* myDog = AnimalFactory::createAnimal("dog");
    Animal* myCat = AnimalFactory::createAnimal("cat");

    if (myDog) myDog->makeSound();
    if (myCat) myCat->makeSound();

    delete myDog;
    delete myCat;
    return 0;
}

