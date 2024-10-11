#include <iostream>

class Singleton {
private:
    static Singleton* instance;

    // 构造函数私有，防止外部实例化
    Singleton() {}

public:
    // 禁止复制和赋值
    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;

    // 获取实例
    static Singleton* getInstance() {
        if (instance == nullptr) {
            instance = new Singleton();
        }
        return instance;
    }

    void showMessage() {
        std::cout << "This is a Singleton instance." << std::endl;
    }
};

// 初始化静态成员
Singleton* Singleton::instance = nullptr;

int main() {
    Singleton* s1 = Singleton::getInstance();
    Singleton* s2 = Singleton::getInstance();

    s1->showMessage();
    
    // 检查是否是同一个实例
    if (s1 == s2) {
        std::cout << "Both are the same instance." << std::endl;
    }
    return 0;
}

