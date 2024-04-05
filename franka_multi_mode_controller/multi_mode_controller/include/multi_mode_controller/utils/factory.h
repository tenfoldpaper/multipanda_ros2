#pragma once

#include <exception>
#include <memory>
#include <mutex>
#include <map>

#include <multi_mode_controller/utils/type_name.h>

namespace panda_controllers {

template <class Base, class... ConstructorArgs>
class Factory {
 public:
  struct Void {
    Void(); // this creates a symbol that we will define in the library, but not
            // inside this template
  };
  static std::unique_ptr<Base> create(const std::string& name,
                                      ConstructorArgs... args) {
    static Void v; // this ensures, that we depend on a symbol from the library
                   // such that it is loaded and executes all registrations at
                   // start-up
    std::lock_guard<std::recursive_mutex> lock(mapMutex());
    auto it = types().find(name);
    if (it != types().end()) {
      return it->second(std::forward<ConstructorArgs>(args)...);
    } else {
      throw std::invalid_argument("Factory<" + typeName<Base>() + ">: Class " +
          name + " does not exist.");
    }
  }
  template <class Derived>
  static Void registerClass(const std::string& name) {
    std::lock_guard<std::recursive_mutex> lock(mapMutex());
    if (types().count(name)) {
      throw std::invalid_argument("Factory<" + typeName<Base>() + ">: Class " +
          name + " of type " + typeName<Derived>() + " already defined.");
    } else {
      types()[name] = &Factory::produce<Derived>;
    }
    return Void();
  }
 private:
  template <class Derived>
  static std::unique_ptr<Base> produce(ConstructorArgs... args) {
    return std::make_unique<Derived>(std::forward<ConstructorArgs>(args)...);
  }
  static std::recursive_mutex& mapMutex() {
    static std::recursive_mutex mtx;
    return mtx;
  }
  static std::map<std::string, std::unique_ptr<Base>(*)(ConstructorArgs...)>&
      types() {
    static std::map<std::string, std::unique_ptr<Base>(*)(ConstructorArgs...)>
        map;
    return map;
  }
};

} // namespace panda_controllers
