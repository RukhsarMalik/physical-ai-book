// physical-ai-book/src/components/AgentThinking.tsx
import React from 'react';

const AgentThinking: React.FC = () => {
  return (
    <div style={{
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'flex-start',
      padding: '10px',
      color: '#666',
      fontSize: '0.9em',
    }}>
      <span style={{ marginRight: '5px' }}>AI is analyzing the textbook...</span>
      <span className="dots">
        <span>.</span><span>.</span><span>.</span>
      </span>
      {/* Basic CSS for animation - ideally this would be in a CSS file */}
      <style>{`
        .dots span {
          animation: blink 1.4s infinite;
        }
        .dots span:nth-child(2) {
          animation-delay: 0.2s;
        }
        .dots span:nth-child(3) {
          animation-delay: 0.4s;
        }
        @keyframes blink {
          0%, 80%, 100% { opacity: 0; }
          40% { opacity: 1; }
        }
      `}</style>
    </div>
  );
};

export default AgentThinking;