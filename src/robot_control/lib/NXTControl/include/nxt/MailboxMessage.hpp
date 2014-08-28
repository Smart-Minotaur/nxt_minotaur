#ifndef NXT_MAILBOX_MESSAGE_HPP
#define NXT_MAILBOX_MESSAGE_HPP

#include <cstdint>
#include <string>

namespace nxt
{
	/**
	 * \brief Data structure for ReadMailbox responses.
	 */
	class MailboxMessage
	{
	public:
		uint8_t status;
		uint8_t mailboxNumber;
		std::string message;
		
		MailboxMessage() { }
		~MailboxMessage() { }
	};
}

#endif