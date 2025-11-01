#include "pcap_listener.hpp"

#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/ether.h>
#include <limits>
#include <unistd.h>
#include <fcntl.h>

/******************************************************************************/
PcapListener::PcapListener(
  rclcpp::Node * node,
  const std::string & pcap_file,
  const std::string & filter_ip)
: node_(node),
  pcap_file_(pcap_file),
  filter_ip_(filter_ip),
  pcap_handle_(nullptr),
  last_pcap_timestamp_(0.0),
  first_pcap_timestamp_(0.0),
  playback_speed_(1.0),
  paused_(false),
  seek_target_(0.0),
  seek_requested_(false),
  tty_fd_(-1)
{
  setupTerminal();
}

/******************************************************************************/
PcapListener::~PcapListener()
{
  stop();
  restoreTerminal();
  if (pcap_handle_) {
    pcap_close(pcap_handle_);
  }
}

/******************************************************************************/
void PcapListener::stop()
{
  running_ = false;
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
  DataListener::stop();
}

/******************************************************************************/
void PcapListener::setupTerminal()
{
  // Open /dev/tty directly (stdin may not be connected in ros2 launch)
  tty_fd_ = ::open("/dev/tty", O_RDWR | O_NONBLOCK);
  if (tty_fd_ < 0) {
    // Can't open TTY, controls won't work
    return;
  }
  
  // Save original terminal settings
  tcgetattr(tty_fd_, &orig_termios_);
  
  // Set terminal to raw mode for immediate key input
  struct termios raw = orig_termios_;
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 1;
  tcsetattr(tty_fd_, TCSANOW, &raw);
}

/******************************************************************************/
void PcapListener::restoreTerminal()
{
  // Restore original terminal settings
  if (tty_fd_ >= 0) {
    tcsetattr(tty_fd_, TCSANOW, &orig_termios_);
    close(tty_fd_);
    tty_fd_ = -1;
  }
}

/******************************************************************************/
void PcapListener::keyboardInputThread()
{
  if (tty_fd_ < 0) {
    // TTY not available, controls disabled
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "\n"
    "=======================================================\n"
    "  PCAP Playback Controls:\n"
    "  SPACE     - Pause/Resume\n"
    "  UP        - Increase speed (0.10x increments)\n"
    "  DOWN      - Decrease speed (0.10x increments)\n"
    "  LEFT (<)  - Seek backward 5 seconds\n"
    "  RIGHT (>) - Seek forward 5 seconds\n"
    "  q         - Quit\n"
    "=======================================================\n");

  while (running_) {
    char c;
    if (read(tty_fd_, &c, 1) == 1) {
      if (c == ' ') {
        // Toggle pause
        bool was_paused = paused_.load();
        paused_.store(!was_paused);
        RCLCPP_INFO(node_->get_logger(), 
          was_paused ? "[RESUME] Playback resumed" : "[PAUSE] Playback paused");
      }
      else if (c == 27) {
        // Escape sequence (arrow keys)
        char seq[2];
        if (read(tty_fd_, &seq[0], 1) == 1 && 
            read(tty_fd_, &seq[1], 1) == 1) {
          if (seq[0] == '[') {
            switch (seq[1]) {
              case 'A': {
                // Up arrow - increase speed
                double current_speed = playback_speed_.load();
                double new_speed = std::min(current_speed + 0.10, 10.0);
                playback_speed_.store(new_speed);
                RCLCPP_INFO(node_->get_logger(), 
                  "[SPEED] Playback speed: %.2fx", new_speed);
                break;
              }
              case 'B': {
                // Down arrow - decrease speed
                double current_speed = playback_speed_.load();
                double new_speed = std::max(current_speed - 0.10, 0.10);
                playback_speed_.store(new_speed);
                RCLCPP_INFO(node_->get_logger(), 
                  "[SPEED] Playback speed: %.2fx", new_speed);
                break;
              }
              case 'D': {
                // Left arrow - seek backward 5 seconds
                double current_relative = last_pcap_timestamp_ - first_pcap_timestamp_;
                double target_relative = std::max(0.0, current_relative - 5.0);
                seek_target_.store(target_relative);
                seek_requested_.store(true);
                RCLCPP_INFO(node_->get_logger(), 
                  "[SEEK] Seeking backward 5s to %.1fs", target_relative);
                break;
              }
              case 'C': {
                // Right arrow - seek forward 5 seconds
                double current_relative = last_pcap_timestamp_ - first_pcap_timestamp_;
                double target_relative = current_relative + 5.0;
                seek_target_.store(target_relative);
                seek_requested_.store(true);
                RCLCPP_INFO(node_->get_logger(), 
                  "[SEEK] Seeking forward 5s to %.1fs", target_relative);
                break;
              }
            }
          }
        }
      }
      else if (c == 'q' || c == 'Q') {
        RCLCPP_INFO(node_->get_logger(), "[QUIT] Stopping PCAP playback...");
        running_ = false;
        break;
      }
      else if (c == '<' || c == ',') {
        // Also support < key for backward seek
        double current_relative = last_pcap_timestamp_ - first_pcap_timestamp_;
        double target_relative = std::max(0.0, current_relative - 5.0);
        seek_target_.store(target_relative);
        seek_requested_.store(true);
        RCLCPP_INFO(node_->get_logger(), 
          "[SEEK] Seeking backward 5s to %.1fs", target_relative);
      }
      else if (c == '>' || c == '.') {
        // Also support > key for forward seek
        double current_relative = last_pcap_timestamp_ - first_pcap_timestamp_;
        double target_relative = current_relative + 5.0;
        seek_target_.store(target_relative);
        seek_requested_.store(true);
        RCLCPP_INFO(node_->get_logger(), 
          "[SEEK] Seeking forward 5s to %.1fs", target_relative);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

/******************************************************************************/
void PcapListener::setCallback(
  const std::function<void(uint8_t *, size_t)> & func)
{
  callback_function_ = func;
}

/******************************************************************************/
int PcapListener::open()
{
  char errbuf[PCAP_ERRBUF_SIZE];
  
  pcap_handle_ = pcap_open_offline(pcap_file_.c_str(), errbuf);
  if (!pcap_handle_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error opening PCAP file '%s': %s",
      pcap_file_.c_str(), errbuf);
    return 1;
  }

  if (!filter_ip_.empty()) {
    struct bpf_program fp;
    std::string filter_str = "host " + filter_ip_;
    
    if (pcap_compile(pcap_handle_, &fp, filter_str.c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Error compiling PCAP filter '%s': %s",
        filter_str.c_str(), pcap_geterr(pcap_handle_));
      return 2;
    }

    if (pcap_setfilter(pcap_handle_, &fp) == -1) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Error setting PCAP filter: %s",
        pcap_geterr(pcap_handle_));
      pcap_freecode(&fp);
      return 3;
    }

    pcap_freecode(&fp);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "PCAP filter set to: %s", filter_str.c_str());
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Opened PCAP file: %s",
    pcap_file_.c_str());

  return 0;
}/******************************************************************************/
bool PcapListener::extractUdpPayload(
  const u_char * packet,
  const struct pcap_pkthdr * header,
  const u_char ** payload_out,
  size_t * payload_len_out)
{
  const size_t ethernet_header_len = 14;
  
  if (header->caplen < ethernet_header_len) {
    return false;
  }

  const struct ip * ip_header = reinterpret_cast<const struct ip *>(packet + ethernet_header_len);
  if (ip_header->ip_v != 4) {
    return false;
  }

  size_t ip_header_len = ip_header->ip_hl * 4;
  if (ip_header->ip_p != IPPROTO_UDP) {
    return false;
  }

  const size_t udp_header_offset = ethernet_header_len + ip_header_len;
  if (header->caplen < udp_header_offset + sizeof(struct udphdr)) {
    return false;
  }

  const struct udphdr * udp_header = 
    reinterpret_cast<const struct udphdr *>(packet + udp_header_offset);

  size_t udp_payload_offset = udp_header_offset + sizeof(struct udphdr);
  size_t udp_payload_len = ntohs(udp_header->len) - sizeof(struct udphdr);

  if (header->caplen < udp_payload_offset + udp_payload_len) {
    udp_payload_len = header->caplen - udp_payload_offset;
  }

  *payload_out = packet + udp_payload_offset;
  *payload_len_out = udp_payload_len;

  return true;
}

/******************************************************************************/
bool PcapListener::extractTcpPayload(
  const u_char * packet,
  const struct pcap_pkthdr * header,
  const u_char ** payload_out,
  size_t * payload_len_out)
{
  const size_t ethernet_header_len = 14;
  
  if (header->caplen < ethernet_header_len) {
    return false;
  }

  const struct ip * ip_header = reinterpret_cast<const struct ip *>(packet + ethernet_header_len);
  if (ip_header->ip_v != 4) {
    return false;
  }

  size_t ip_header_len = ip_header->ip_hl * 4;
  if (ip_header->ip_p != IPPROTO_TCP) {
    return false;
  }

  const size_t tcp_header_offset = ethernet_header_len + ip_header_len;
  if (header->caplen < tcp_header_offset + sizeof(struct tcphdr)) {
    return false;
  }

  const struct tcphdr * tcp_header = 
    reinterpret_cast<const struct tcphdr *>(packet + tcp_header_offset);

  size_t tcp_header_len = tcp_header->doff * 4;
  size_t tcp_payload_offset = tcp_header_offset + tcp_header_len;
  size_t total_ip_len = ntohs(ip_header->ip_len);
  size_t tcp_payload_len = total_ip_len - ip_header_len - tcp_header_len;

  if (header->caplen < tcp_payload_offset) {
    return false;
  }

  if (header->caplen < tcp_payload_offset + tcp_payload_len) {
    tcp_payload_len = header->caplen - tcp_payload_offset;
  }

  if (tcp_payload_len == 0) {
    return false;
  }

  *payload_out = packet + tcp_payload_offset;
  *payload_len_out = tcp_payload_len;

  return true;
}

/******************************************************************************/
void PcapListener::listen()
{
  if (open() != 0) {
    return;
  }

  running_ = true;
  last_pcap_timestamp_ = 0.0;
  
  // Start keyboard input thread
  keyboard_thread_ = std::thread(&PcapListener::keyboardInputThread, this);
  
  struct pcap_pkthdr * header;
  const u_char * packet;
  int packet_count = 0;
  int processed_count = 0;
  int tcp_udp_count = 0;
  
  auto playback_start_time = std::chrono::steady_clock::now();
  double pcap_start_time = 0.0;
  bool first_packet = true;
  double last_speed = 1.0;

  RCLCPP_INFO(node_->get_logger(), "Starting PCAP playback at 1.0x speed...");

  try {
    int pcap_result;
    while (running_ && (pcap_result = pcap_next_ex(pcap_handle_, &header, &packet)) >= 0) {
      if (pcap_result == 0) {
        continue;
      }
      
      packet_count++;
      
      // Get packet timestamp
      double current_pcap_timestamp = 
        header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
      
      if (first_packet) {
        pcap_start_time = current_pcap_timestamp;
        first_pcap_timestamp_ = current_pcap_timestamp;
        first_packet = false;
      }
      
      // Check for seek request
      if (seek_requested_.load()) {
        double target_relative = seek_target_.load();
        double target_absolute = first_pcap_timestamp_ + target_relative;
        seek_requested_.store(false);
        
        // If seeking backward, restart from beginning
        if (target_absolute < current_pcap_timestamp) {
          pcap_close(pcap_handle_);
          if (open() != 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to reopen PCAP file");
            break;
          }
          packet_count = 0;
          processed_count = 0;
          tcp_udp_count = 0;
          first_packet = true;
          playback_start_time = std::chrono::steady_clock::now();
          
          // Skip forward to target position
          while (running_ && (pcap_result = pcap_next_ex(pcap_handle_, &header, &packet)) >= 0) {
            if (pcap_result == 0) continue;
            
            double ts = header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
            
            // Set first packet timestamp
            if (first_packet) {
              first_pcap_timestamp_ = ts;
              pcap_start_time = ts;
              first_packet = false;
              target_absolute = first_pcap_timestamp_ + target_relative;
            }
            
            if (ts >= target_absolute) {
              current_pcap_timestamp = ts;
              playback_start_time = std::chrono::steady_clock::now();
              pcap_start_time = current_pcap_timestamp;
              last_pcap_timestamp_ = current_pcap_timestamp;
              double relative_time = current_pcap_timestamp - first_pcap_timestamp_;
              RCLCPP_INFO(node_->get_logger(), 
                "[SEEK] Jumped to %.1fs (relative time in PCAP)", relative_time);
              break;
            }
          }
          continue;
        }
        
        // For forward seek, skip packets until we reach target
        if (target_absolute > current_pcap_timestamp) {
          while (running_ && (pcap_result = pcap_next_ex(pcap_handle_, &header, &packet)) >= 0) {
            if (pcap_result == 0) continue;
            
            double ts = header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
            if (ts >= target_absolute) {
              current_pcap_timestamp = ts;
              playback_start_time = std::chrono::steady_clock::now();
              pcap_start_time = current_pcap_timestamp;
              last_pcap_timestamp_ = current_pcap_timestamp;
              double relative_time = current_pcap_timestamp - first_pcap_timestamp_;
              RCLCPP_INFO(node_->get_logger(), 
                "[SEEK] Jumped to %.1fs (relative time in PCAP)", relative_time);
              break;
            }
          }
          continue;
        }
      }

      // Extract payload
      const u_char * payload;
      size_t payload_len;
      
      if (!extractTcpPayload(packet, header, &payload, &payload_len) &&
          !extractUdpPayload(packet, header, &payload, &payload_len)) {
        continue;
      }
      
      tcp_udp_count++;
      
      // Handle pause
      while (paused_.load() && running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Reset timing when resuming
        playback_start_time = std::chrono::steady_clock::now();
        pcap_start_time = current_pcap_timestamp;
      }
      
      if (!running_) break;
      
      // Handle playback timing with speed control
      double speed = playback_speed_.load();
      
      // Reset timing if speed changed
      if (std::abs(speed - last_speed) > 0.01) {
        playback_start_time = std::chrono::steady_clock::now();
        pcap_start_time = current_pcap_timestamp;
        last_speed = speed;
      }
      
      if (speed > 0.0 && !first_packet) {
        double pcap_elapsed = current_pcap_timestamp - pcap_start_time;
        double real_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - playback_start_time).count();
        
        double expected_real_time = pcap_elapsed / speed;
        double sleep_time = expected_real_time - real_elapsed;
        
        if (sleep_time > 0.0) {
          std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
      }
      
      last_pcap_timestamp_ = current_pcap_timestamp;
      
      // Process the packet
      if (callback_function_ && payload_len > 0) {
        std::vector<uint8_t> buffer(payload, payload + payload_len);
        callback_function_(buffer.data(), payload_len);
        processed_count++;
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "PCAP playback completed. Processed %d TCP/UDP packets out of %d total packets.",
      processed_count, packet_count);

  } catch (std::exception const & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "PCAP decoder exception: " << ex.what());
  }
  
  running_ = false;
}

/******************************************************************************/
void PcapListener::write(uint8_t * data, size_t size)
{
  // Writing is not supported for PCAP playback
  static_cast<void>(data);
  static_cast<void>(size);
  
  RCLCPP_WARN_ONCE(
    node_->get_logger(),
    "Write operation not supported for PCAP playback");
}
