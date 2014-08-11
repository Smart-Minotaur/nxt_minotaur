#ifndef NXT_CONTROL_NXT_CONTAINER_HPP_
#define NXT_CONTROL_NXT_CONTAINER_HPP_
    
namespace nxtcon
{
    /**
     * \brief Container-class for speed-information
     */
    class TachoData
    {
    public:
        /* Count since last motor reset */
        int32_t tachoCount;
        /* Count since last programmed movement (last command)*/
        int32_t blockTachoCount;
        /* Count since last reset of rotation sensor of this motor*/
        int32_t rotationCount;
        
        TachoData()
        :tachoCount(0), blockTachoCount(0), rotationCount(0) { }
        
        TachoData(const int32_t p_tachoCount, const int32_t p_blocktachoCount, const int32_t p_rotationCount)
        :tachoCount(p_tachoCount), blockTachoCount(p_blocktachoCount), rotationCount(p_rotationCount) { }
        virtual ~TachoData() { }
        
        void set(const int32_t p_tachoCount, const int32_t p_blocktachoCount, const int32_t p_rotationCount)
        {tachoCount = p_tachoCount; blockTachoCount = p_blocktachoCount; rotationCount = p_rotationCount;}
    };
    
    /**
     * \brief Container-class for Sensor-values
     */
    class SensorData
    {
    public:
        int8_t valid;
        int8_t calibrated;
        uint16_t rawValue;
        uint16_t normalizedValue;
        int16_t scaledValue;
        int16_t calibratedValue;
        
        SensorData()
        :valid(false), calibrated(false), rawValue(0), normalizedValue(0), scaledValue(0), calibratedValue(0) { }
        
        virtual ~SensorData() { }
    };
}
    
#endif