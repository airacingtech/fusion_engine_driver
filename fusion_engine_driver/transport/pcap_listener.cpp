#include "pcap_listener.hpp"

#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <netinet/ether.h>
#include <limits>

/******************************************************************************/
PcapListener::PcapListener(
  rclcpp::Node * node,
  const std::string & pcap_file,
  const std::string & filter_ip)
: node_(node),
  pcap_file_(pcap_file),
  filter_ip_(filter_ip),
  pcap_handle_(nullptr),
  last_pcap_timestamp_(0.0)
{
}

/******************************************************************************/
PcapListener::~PcapListener()
{
  if (pcap_handle_) {
    pcap_close(pcap_handle_);
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
    RCLCPP_INFO(
      node_->get_logger(),
      "PCAP filter set to: %s", filter_str.c_str());
  }

  RCLCPP_INFO(
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
  
  struct pcap_pkthdr * header;
  const u_char * packet;
  int packet_count = 0;
  int processed_count = 0;
  int tcp_udp_count = 0;

  try {
    int pcap_result;
    while (running_ && (pcap_result = pcap_next_ex(pcap_handle_, &header, &packet)) >= 0) {
      if (pcap_result == 0) {
        // Timeout occurred (shouldn't happen with offline files)
        continue;
      }
      
      packet_count++;

      const u_char * payload;
      size_t payload_len;
      
      if (extractTcpPayload(packet, header, &payload, &payload_len)) {
        tcp_udp_count++;
      } else if (extractUdpPayload(packet, header, &payload, &payload_len)) {
        tcp_udp_count++;
      } else {
        continue;
      }

      // Handle timing for real-time playback
      double current_pcap_timestamp = 
        header->ts.tv_sec + header->ts.tv_usec / 1000000.0;
      
      if (last_pcap_timestamp_ > 0.0) {
        double pcap_time_diff = current_pcap_timestamp - last_pcap_timestamp_;
        
        if (pcap_time_diff > 0.0) {
          std::this_thread::sleep_for(
            std::chrono::duration<double>(pcap_time_diff));
        }
      }
      
      last_pcap_timestamp_ = current_pcap_timestamp;
      if (callback_function_ && payload_len > 0) {
        // Copy to non-const buffer for callback
        std::vector<uint8_t> buffer(payload, payload + payload_len);
        callback_function_(buffer.data(), payload_len);
        processed_count++;
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "PCAP playback completed. Processed %d packets.",
      processed_count);

  } catch (std::exception const & ex) {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      "PCAP decoder exception: " << ex.what());
  }
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
