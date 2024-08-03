template <typename T>
class Status{
    bool status;
    T* val;

    operator bool() {
        return status;
    }
    T value(){
        static_assert(status, "huiuhiuhiu");
        return *val;
    }
};