import React from "react";
import type { Props } from "@theme/Root";
import { ChatProvider, useChatContext } from "../contexts/ChatContext";
import ChatButton from "../components/ChatButton";
import ChatWindow from "../components/ChatWindow";
import MessageList from "../components/MessageList";
import MessageInput from "../components/MessageInput";
import AgentThinking from "../components/AgentThinking";
import TextSelectionHandler from "../components/TextSelectionHandler"; // Import TextSelectionHandler

/**
 * CLEAN & STABLE Chat UI
 * No selection interference here.
 */

function ChatUI() {
  const {
    isOpen,
    setIsOpen,
    messages,
    sendChatMessage,
    isAgentThinking,
  } = useChatContext();

  return (
    <>
      {/* Handles text selection globally */}
      <TextSelectionHandler />

      {/* Floating Chat Button */}
      <ChatButton onClick={() => setIsOpen(!isOpen)} />

      {/* Chat Window */}
      {isOpen && (
        <ChatWindow isOpen={isOpen} onClose={() => setIsOpen(false)}>
          <MessageList messages={messages} />
          {isAgentThinking && <AgentThinking />}
          {/* ⬇ Chat sends only normal messages */}
          <MessageInput
            onSendMessage={sendChatMessage}
            isLoading={isAgentThinking}
          />
        </ChatWindow>
      )}
    </>
  );
}

/**
 * Root wrapper — keeps ChatProvider at top
 */
export default function Root(props: Props): JSX.Element {
  return (
    <ChatProvider>
      <ChatUI />
      {props.children}
    </ChatProvider>
  );
}
