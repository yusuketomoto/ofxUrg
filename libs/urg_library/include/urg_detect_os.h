#ifndef URG_DETECT_OS_H
#define URG_DETECT_OS_H

/*!
  \file
  \brief OS ‚ÌŒŸo

  \author Satofumi KAMIMURA

  $Id: urg_detect_os.h,v 6c17b740611a 2014/03/24 09:58:01 jun $
*/

#if defined(_WIN32)
#define URG_WINDOWS_OS

#if defined(_MSC_VER) || defined(__BORLANDC__)
#define URG_MSC
#endif

#elif defined(__linux__)
#define URG_LINUX_OS

#else
// ŒŸo‚Å‚«‚È‚¢‚Æ‚«‚ğAMac ˆµ‚¢‚É‚µ‚Ä‚µ‚Ü‚¤
#define URG_MAC_OS
#endif

#endif /* !URG_DETECT_OS_H */
