#include <iostream>
#include <vector>

// 观察者接口
class Observer {
public:
    virtual void update() = 0;
};

// 被观察者
class Subject {
private:
    std::vector<Observer*> observers;

public:
    void addObserver(Observer* observer) {
        observers.push_back(observer);
    }

    void notifyObservers() {
        for (auto& observer : observers) {
            observer->update();
        }
    }
};

// 具体观察者
class ConcreteObserver : public Observer {
public:
    void update() override {
        std::cout << "Observer notified." << std::endl;
    }
};

int main() {
    Subject subject;
    ConcreteObserver observer1, observer2;

    subject.addObserver(&observer1);
    subject.addObserver(&observer2);

    subject.notifyObservers();

    return 0;
}

