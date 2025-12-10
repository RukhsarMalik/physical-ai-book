// physical-ai-book/src/contexts/ChatContext.tsx

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useEffect,
  useRef,
  ReactNode,
} from "react";
import axios, { AxiosInstance } from "axios";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

export type Message = {
  id: string;
  role: "user" | "assistant";
  content: string;
  sources?: string[];
  isStreaming?: boolean;
};

export type ChatContextType = {
  messages: Message[];
  addMessage: (message: Omit<Message, "id">) => void;
  removeLastAssistantMessage: () => void;
  updateLastAssistantMessage: (newContent: string, newSources?: string[]) => void;
  isOpen: boolean;
  setIsOpen: (open: boolean) => void;
  isAgentThinking: boolean;
  setAgentThinking: (thinking: boolean) => void;
  sessionId: string | null;
  setSessionId: (id: string | null) => void;
  selectedText: string | null;
  setSelectedText: (text: string | null) => void;
  sendChatMessage: (message: string) => Promise<void>;
  sendSelectionMessage: (selectedText: string) => Promise<void>;
  api: AxiosInstance;
};

const ChatContext = createContext<ChatContextType | undefined>(undefined);

export const ChatProvider = ({ children }: { children: ReactNode }) => {

  // ⭐ SSR-SAFE FIX — DO NOT CHANGE
  const context = useDocusaurusContext();
  const backendApiUrl =
    (context?.siteConfig?.customFields?.backendApiUrl as string) ||
    "http://localhost:8000";

  console.log("Backend API URL Loaded:", backendApiUrl);

  // Axios client
  const api = axios.create({
    baseURL: backendApiUrl,
    headers: { "Content-Type": "application/json" },
  });

  const [messages, setMessages] = useState<Message[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [isAgentThinking, setAgentThinking] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const welcomeMessageAddedRef = useRef(false);

  // ADD MESSAGE
  const addMessage = useCallback((msg: Omit<Message, "id">) => {
    setMessages((prev) => [
      ...prev,
      { ...msg, id: Date.now().toString() + Math.random() },
    ]);
  }, []);

  // REMOVE LAST ASSISTANT MESSAGE
  const removeLastAssistantMessage = useCallback(() => {
    setMessages((prev) => {
      const last = prev[prev.length - 1];
      if (last?.role === "assistant") return prev.slice(0, -1);
      return prev;
    });
  }, []);

  // UPDATE LAST ASSISTANT MESSAGE
  const updateLastAssistantMessage = useCallback(
    (newContent: string, newSources?: string[]) => {
      setMessages((prev) =>
        prev.map((msg, i) =>
          i === prev.length - 1
            ? {
                ...msg,
                content: newContent,
                sources: newSources,
                isStreaming: false,
              }
            : msg
        )
      );
    },
    []
  );

  // WELCOME MESSAGE
  useEffect(() => {
    if (!welcomeMessageAddedRef.current && messages.length === 0) {
      addMessage({
        role: "assistant",
        content:
          "Hello! I’m your AI assistant for the Physical AI & Humanoid Robotics book. Select any text and ask me about it!",
      });

      welcomeMessageAddedRef.current = true;
    }
  }, [messages.length, addMessage]);

  // NORMAL CHAT MESSAGE
  const sendChatMessage = useCallback(
    async (message: string) => {
      addMessage({ role: "user", content: message });
      setAgentThinking(true);

      addMessage({ role: "assistant", content: "", isStreaming: true });

      try {
        const res = await api.post("/api/chat", {
          message,
          session_id: sessionId,
        });

        const data = res.data;

        updateLastAssistantMessage(data.answer, data.sources);

        if (data.session_id && !sessionId) {
          setSessionId(data.session_id);
        }
      } catch (e) {
        console.error("Chat error:", e);
        removeLastAssistantMessage();

        addMessage({
          role: "assistant",
          content: "Oops, something went wrong. Please try again!",
        });
      } finally {
        setAgentThinking(false);
      }
    },
    [addMessage, api, sessionId, updateLastAssistantMessage, removeLastAssistantMessage, setSessionId]
  );

  // SELECTION MESSAGE
  const sendSelectionMessage = useCallback(
    async (text: string) => {
      if (!text) return;

      addMessage({
        role: "user",
        content: `📌 Selected Text:\n\`\`\`\n${text}\n\`\`\``,
      });

      addMessage({
        role: "user",
        content: "Explain this part.",
      });

      const formattedMessage = `SELECTED_TEXT:
"""${text}"""

Explain this part.`;

      setAgentThinking(true);
      addMessage({ role: "assistant", content: "", isStreaming: true });

      try {
        const res = await api.post("/api/chat", {
          message: formattedMessage,
          session_id: sessionId,
        });

        const data = res.data;

        updateLastAssistantMessage(data.answer, data.sources);

        if (data.session_id && !sessionId) {
          setSessionId(data.session_id);
        }
      } catch (e) {
        console.error("Selection chat error:", e);
        removeLastAssistantMessage();

        addMessage({
          role: "assistant",
          content:
            "Sorry, I couldn’t process that selected text. Try again!",
        });
      } finally {
        setAgentThinking(false);
      }
    },
    [addMessage, api, sessionId, updateLastAssistantMessage, removeLastAssistantMessage, setSessionId]
  );

  return (
    <ChatContext.Provider
      value={{
        messages,
        addMessage,
        removeLastAssistantMessage,
        updateLastAssistantMessage,
        isOpen,
        setIsOpen,
        isAgentThinking,
        setAgentThinking,
        sessionId,
        setSessionId,
        selectedText,
        setSelectedText,
        sendChatMessage,
        sendSelectionMessage,
        api,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
};

export const useChatContext = () => {
  const ctx = useContext(ChatContext);
  if (!ctx) throw new Error("useChatContext must be used inside ChatProvider");
  return ctx;
};
