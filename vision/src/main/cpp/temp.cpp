@Jaci#2393 YOU HAVE BEEN SUMMONED
It's locking but it's not unlocking...?


Processing Block
```cpp
void BallProcessing::Init() {
  std::cout << "BallProcessing Init Started" << std::endl;
  Process::Init();
  std::cout << "BallProcessing Init Ended" << std::endl;
}

void BallProcessing::Periodic() {
  std::cout << "BallProcessing Periodic Started" << std::endl;
  

  {
    std::lock_guard<std::mutex> lk(_classMutex);
    _ready = true;
    std::cout << "main() signals data ready for processing\n";
  }
  _conVar.notify_all();
 
  // wait for the worker
  {
    std::unique_lock<std::mutex> lk(_classMutex);
    _conVar.wait(lk);
  }
 ```

```cpp
Init Block
void Process::Init() {
  
  std::cout << "Process Init Started" << std::endl;
  // Wait until Periodic() sends data
  std::unique_lock<std::mutex> lk(_classMutex);
  _conVar.wait(lk);
 
  // after the wait, we own the lock.
  std::cout << "Worker thread is processing data\n";
  
  _videoMode = _capture.GetVideoMode();
  _imgTrack = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
  _imgOriginal = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
 
  // Send data back to Init()
  _processed = true;
  std::cout << "Worker thread signals data processing completed\n";
 
  // Manual unlocking is done before notifying, to avoid waking up
  // the waiting thread only to block again (see notify_one for details)
  lk.unlock();
  _conVar.notify_all();
  

  std::cout << "Process Init Ended" << std::endl;
}
```