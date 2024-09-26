#include <Zivid/Zivid.h>  // Include the Zivid SDK
#include <chrono>
#include <thread>

// Warm-up function to capture frames for a few minutes
void warmupCamera(Zivid::Camera& camera, int durationInSeconds)
{
    // Define settings with an acquisition (required)
    Zivid::Settings settings;
    settings.set(Zivid::Settings::Acquisitions{    
        Zivid::Settings::Acquisition{        
            Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{10000} },  // Example: 10 ms exposure time        
            Zivid::Settings::Acquisition::Aperture{ 5.6 }  // Example: aperture    
        }
    });

    for (int i = 0; i < durationInSeconds; ++i)
    {
        auto frame = camera.capture(settings);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main()
{
    Zivid::Application zivid;
    auto camera = zivid.connectCamera();

    // Warm up the camera for 5 minutes
    warmupCamera(camera, 300);
}
