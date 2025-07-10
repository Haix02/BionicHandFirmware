/**
 * @file Finger.h
 * @brief Enhanced finger control with reflex capabilities
 */

// Existing code
// ...

// Reflex parameters
#define REFLEX_BOOST_PERCENT 15    // Boost grip by 15% of remaining range
#define REFLEX_MIN_BOOST 5         // Minimum position boost
#define REFLEX_DEBOUNCE_MS 300     // Minimum time between reflexes
#define REFLEX_DURATION_MS 1000    // How long reflex boost remains active

class Finger {
public:
    // Existing declarations
    // ...
    
    // Add reflexGrip method
    void reflexGrip();
    
private:
    // Existing member variables
    // ...
    
    // Add reflex-related members
    uint32_t _lastReflexTime = 0;  // Time of last reflex activation
    bool _reflexActive = false;    // Whether reflex is currently active
    uint16_t _preReflexTarget = 0; // Target position before reflex
};