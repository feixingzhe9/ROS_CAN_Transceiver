#ifndef H_CANMSGBUFFER
#define H_CANMSGBUFFER

#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

struct _cout_wrapper{
    static boost::mutex& get_cout_mutex(){
        static boost::mutex mutex;
        return mutex;
    }
};

#define LOG(log) { boost::mutex::scoped_lock _cout_lock(_cout_wrapper::get_cout_mutex()); std::cout << log << std::endl; }

namespace can {

class CanMsgBuffer {
    std::deque<CanMsg> _buffer;
    boost::mutex _mutex;
    boost::condition_variable _cond;
    bool _enabled;
    size_t _max_len;

    void trim() {
        if ( _max_len > 0 ) {
            while ( _buffer.size() > _max_len ) {
                LOG("buffer overflow, discarded oldest message");
                _buffer.pop_front();
            }
        }
    }

public:
    bool handleCanMsg(const can::CanMsg &msg) {
        boost::mutex::scoped_lock lock(_mutex);
        if (_enabled) {
            _buffer.push_back(msg);
            trim();
            _cond.notify_one();
            return true;
        } else {
            LOG("discarded message");
            return false;
        }
    }
    
    CanMsgBuffer(): _enabled(true), _max_len(0) {}
    CanMsgBuffer(bool enable, size_t max_len = 0) : _enabled(enable), _max_len(max_len) {}

    void flush() {
        boost::mutex::scoped_lock lock(_mutex);
        _buffer.clear();
    }

    void setMaxLen(size_t max_len) {
        boost::mutex::scoped_lock lock(_mutex);
        _max_len = max_len;
        trim();
    }

    bool isEnabled() {
        boost::mutex::scoped_lock lock(_mutex);
        return _enabled;
    }

    bool setEnabled(bool enabled) {
        boost::mutex::scoped_lock lock(_mutex);
        bool before = _enabled;
        _enabled = enabled;
        return before;
    }

    void enable() {
        boost::mutex::scoped_lock lock(_mutex);
        _enabled = true;
    }

    void disable() {
        boost::mutex::scoped_lock lock(_mutex);
        _enabled = false;
    }

    size_t getBufferSize() {
        boost::mutex::scoped_lock lock(_mutex);
        return _buffer.size();
    }

    template<typename DurationType> bool read(can::CanMsg * msg, const DurationType &duration){
        return readUntil(msg, boost::chrono::high_resolution_clock::now() + duration);
    }

    bool readUntil(can::CanMsg * msg, boost::chrono::high_resolution_clock::time_point abs_time){
        boost::mutex::scoped_lock lock(_mutex);

        while(_buffer.empty() && _cond.wait_until(lock,abs_time)  != boost::cv_status::timeout)
        {}

        if(_buffer.empty()){
            return false;
        }

        if(msg){
            *msg = _buffer.front();
            _buffer.pop_front();
        }
        return true;
    }

    bool read(can::CanMsg *msg) {
        boost::mutex::scoped_lock lock(_mutex);

        if(_buffer.empty()){
            return false;
        }

        if(msg){
            *msg = _buffer.front();
            _buffer.pop_front();
        }
        return true;
    }
};

} //namespace can

#endif
