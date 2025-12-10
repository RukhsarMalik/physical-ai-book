import axios from "axios";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

// Client-only baseURL detection
const getApiBaseUrl = () => {
  if (typeof window !== "undefined") {
    const { siteConfig } = useDocusaurusContext();
    return siteConfig.customFields.backendApiUrl as string;
  }

  // NEVER use process.env on client
  return "http://localhost:8000";
};

const api = axios.create({
  headers: {
    "Content-Type": "application/json",
  },
});

export const useApi = () => {
  api.defaults.baseURL = getApiBaseUrl();
  return api;
};

// Chat API
export const sendMessage = async (message: string, sessionId?: string) => {
  const api = useApi();
  const response = await api.post("/api/chat", {
    message,
    session_id: sessionId,
  });
  return response.data;
};
