// physical-ai-book/src/components/RobotIcon.tsx
import React from 'react';

const RobotIcon = (props: React.SVGProps<SVGSVGElement>) => (
    <svg
        width="24"
        height="24"
        viewBox="0 0 24 24"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
        {...props}
    >
        <path
            d="M12 1C5.92487 1 1 5.92487 1 12C1 14.3731 1.7232 16.5671 3 18.3333L1 23L5.66667 21C7.4329 22.2768 9.62689 23 12 23C18.0751 23 23 18.0751 23 12C23 5.92487 18.0751 1 12 1Z"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
        />
        <path
            d="M8 10H16"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
        />
        <path
            d="M8 14H16"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
        />
    </svg>
);

export default RobotIcon;