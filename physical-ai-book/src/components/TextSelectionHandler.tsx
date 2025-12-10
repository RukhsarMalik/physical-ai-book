import React, { useEffect, useCallback, useRef, useState } from "react";
import { useChatContext } from "../contexts/ChatContext";

const TextSelectionHandler: React.FC = () => {
  const { sendSelectionMessage, setSelectedText: setChatContextSelectedText } = useChatContext();

  const [tooltipVisible, setTooltipVisible] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });
  const [selectedContent, setSelectedContent] = useState<string | null>(null);
  const tooltipRef = useRef<HTMLDivElement>(null);

  // 📌 Handle text selection
  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length > 10) {
      setSelectedContent(text);
      setChatContextSelectedText(text);

      const range = selection?.getRangeAt(0);
      if (range) {
        const rect = range.getBoundingClientRect();
        setTooltipPosition({
          x: rect.left + window.scrollX + rect.width / 2,
          y: rect.top + window.scrollY - 40,
        });
        setTooltipVisible(true);
      }
    } else {
      setTooltipVisible(false);
      setSelectedContent(null);
      setChatContextSelectedText(null);
    }
  }, [setChatContextSelectedText]);

  // 📌 When tooltip is clicked → auto-send selected text
  const handleTooltipClick = useCallback(async () => {
    if (selectedContent) {
      setTooltipVisible(false);

      // ❗ DO NOT OPEN SIDEBAR
      // setIsOpen(true);

      // ⭐ Auto send selected text
      await sendSelectionMessage(selectedContent);

      // Clear selection
      window.getSelection()?.removeAllRanges();
      setSelectedContent(null);
      setChatContextSelectedText(null);
    }
  }, [selectedContent, sendSelectionMessage, setChatContextSelectedText]);

  // detect user selection
  useEffect(() => {
    document.addEventListener("mouseup", handleSelection);
    return () => document.removeEventListener("mouseup", handleSelection);
  }, [handleSelection]);

  // close tooltip on outside click
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (tooltipRef.current && !tooltipRef.current.contains(e.target as Node)) {
        setTooltipVisible(false);
      }
    };
    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, []);

  return (
    <>
      {tooltipVisible && selectedContent && (
        <div
          ref={tooltipRef}
          style={{
            position: "absolute",
            left: tooltipPosition.x,
            top: tooltipPosition.y,
            transform: "translateX(-50%)",
            backgroundColor: "#333",
            color: "white",
            padding: "6px 12px",
            borderRadius: "6px",
            fontSize: "0.8rem",
            cursor: "pointer",
            whiteSpace: "nowrap",
            zIndex: 1001,
          }}
          onClick={handleTooltipClick}
        >
          Ask AI about this section
        </div>
      )}
    </>
  );
};

export default TextSelectionHandler;
