#include <Arduino.h> 
#include <shared.h>

class BatteryFilter {
public:
    BatteryFilter(float alpha = 0.05f) : _alpha(alpha), _initialized(false) {}

    float update(float raw) {
        if (!_initialized) {
            _value = raw;
            _initialized = true;
        } else {
            _value = _alpha * raw + (1.0f - _alpha) * _value;
        }
        return _value;
    }

    float value() const { return _value; }

private:
    float _alpha;
    float _value = 0;
    bool  _initialized;
};
