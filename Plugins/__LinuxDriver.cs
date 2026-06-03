namespace Plugins
{
    public class __LinuxDriver
    {
        /*
         We have 2 options:

         - https://github.com/wheaney/breezy-desktop?tab=readme-ov-file

           can be installed with a single sh: https://github.com/wheaney/breezy-desktop/releases/latest/download/breezy_vulkan_setup

           installation log:

           peng@pop-os:~$ chmod +x ~/Downloads/breezy_vulkan_setup
           peng@pop-os:~$ sudo ~/Downloads/breezy_vulkan_setup
           [sudo] password for peng:
           Created temp directory: /tmp/breezy-vulkan-Hli2GycZLU
           Downloading latest release to: /tmp/breezy-vulkan-Hli2GycZLU/breezyVulkan.tar.gz
             % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                            Dload  Upload   Total   Spent    Left  Speed
             0     0    0     0    0     0      0      0 --:--:-- --:--:-- --:--:--     0
             0     0    0     0    0     0      0      0 --:--:-- --:--:-- --:--:--     0
           100 1855k  100 1855k    0     0  2724k      0 --:--:-- --:--:-- --:--:-- 2724k
           Extracting to: /tmp/breezy-vulkan-Hli2GycZLU/breezy_vulkan
           Copying the breezy_vulkan scripts to /home/peng/bin
           Installing vkBasalt; copying binaries, configs, and shader files to /home/peng/.local and /home/peng/.config
           Installing the Sombrero shaders and texture files to /home/peng/.config/reshade/{Shaders,Textures}
           Installing xrealAirLinuxDriver
           BEGIN - xreal_driver_setup
           Created temp directory: /tmp/xreal-air-7RPMtJBecQ
           Extracting to: /tmp/xreal-air-7RPMtJBecQ/driver_air_glasses
           Setting up uinput kernel module
           Copying driver binary and scripts to /home/peng/bin
           Setting up the systemd service
           Created symlink /etc/systemd/system/multi-user.target.wants/xreal-air-driver.service → /etc/systemd/system/xreal-air-driver.service.
           Deleting temp directory: /tmp/xreal-air-7RPMtJBecQ
           END - xreal_driver_setup
           Deleting temp directory: /tmp/breezy-vulkan-Hli2GycZLU

         */
    }
}