
// Modern Chart Configuration
Chart.defaults.color = '#94a3b8';
Chart.defaults.font.family = "'Sarabun', sans-serif";
Chart.defaults.scale.grid.color = 'rgba(255, 255, 255, 0.05)';

document.addEventListener('DOMContentLoaded', async () => {
    const ctxOrigin = document.getElementById('originChart').getContext('2d');
    const ctxProvince = document.getElementById('provinceChart').getContext('2d');
    const ctxInterest = document.getElementById('interestChart').getContext('2d');
    const totalEl = document.getElementById('total-conversations');
    const loader = document.getElementById('loader');

    let originChartInstance = null;
    let provinceChartInstance = null;
    let interestChartInstance = null;

    async function fetchData() {
        try {
            const response = await fetch(`${API_BASE_URL}/api/admin/analytics/dashboard?days=30`);
            if (!response.ok) throw new Error("Failed to fetch data");
            const data = await response.json();

            // Active Connection Pulse (Visual Feedback)
            const brandIcon = document.querySelector('.brand i');
            if (brandIcon) { // Check if brandIcon exists
                brandIcon.style.color = '#4ade80';
                setTimeout(() => brandIcon.style.color = 'white', 500);
            }

            // Animate Number (Only if changed significantly, or just set it to avoid constant jumping)
            // For real-time updates, direct setting is often smoother/less distracting than re-animating from 0
            totalEl.textContent = data.total_conversations.toLocaleString();

            // Update Top Location (ใช้ location_stats แทน interest_stats)
            const topLocationEl = document.getElementById('top-location');
            const topLocationCountEl = document.getElementById('top-location-count');

            if (data.location_stats && data.location_stats.length > 0) {
                const topLocation = data.location_stats[0];
                topLocationEl.textContent = topLocation._id || "ไม่มีข้อมูล";
                topLocationCountEl.innerHTML = `<i class="fa-solid fa-fire"></i> ถูกถามถึง ${topLocation.count} ครั้ง`;
                topLocationCountEl.innerHTML = `<i class="fa-solid fa-info-circle"></i> ลองถามน้องน่านเกี่ยวกับสถานที่ต่างๆ`;
            }

            // 🧹 [Cleanup] Removed temporary Satisfaction Meter logic

            // 🆕 Satisfaction Score (5-Star System)
            if (data.feedback_stats) {
                const likeCount = (data.feedback_stats.find(i => i._id === 'like') || {}).count || 0;
                const dislikeCount = (data.feedback_stats.find(i => i._id === 'dislike') || {}).count || 0;
                const totalFeedback = likeCount + dislikeCount;

                let score = 5.0; // Default
                let label = "ยอดเยี่ยม";
                let colorClass = "text-emerald-400"; // Tailwind class assumption or style

                if (totalFeedback > 0) {
                    score = (likeCount / totalFeedback) * 5.0;
                }

                // Determine Label & Color
                if (score >= 4.5) {
                    label = "ยอดเยี่ยม";
                    colorClass = "#4ade80"; // Bright Green
                } else if (score >= 3.0) {
                    label = "ดี";
                    colorClass = "#fbbf24"; // Amber/Yellow
                } else {
                    label = "ต้องปรับปรุง";
                    colorClass = "#f87171"; // Red
                }

                // Update UI elements
                const scoreEl = document.getElementById('satisfaction-score');
                const labelEl = document.getElementById('satisfaction-label');
                const containerEl = document.getElementById('satisfaction-container');

                if (scoreEl) {
                    scoreEl.textContent = score.toFixed(1);
                    // Animate if it's the first load (optional, but nice)
                }

                if (labelEl) labelEl.textContent = label;

                if (containerEl) {
                    containerEl.style.color = colorClass;
                }
            }

            // Origin Chart (Doughnut)
            const originLabels = data.origin_stats.map(item => item._id || "ไม่ระบุ");
            const originData = data.origin_stats.map(item => item.count);

            if (originChartInstance) {
                originChartInstance.data.labels = originLabels;
                originChartInstance.data.datasets[0].data = originData;
                originChartInstance.update('none'); // Update without full re-render
            } else {
                originChartInstance = new Chart(ctxOrigin, {
                    type: 'doughnut',
                    data: {
                        labels: originLabels,
                        datasets: [{
                            data: originData,
                            backgroundColor: [
                                '#38bdf8', '#818cf8', '#c084fc', '#f472b6', '#fb7185',
                                '#22d3ee', '#34d399', '#a78bfa'
                            ],
                            borderWidth: 0,
                            hoverOffset: 10
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        cutout: '70%',
                        plugins: {
                            legend: { position: 'right', labels: { usePointStyle: true, padding: 20 } }
                        }
                    }
                });
            }

            // 🆕 Province Chart (Horizontal Bar) - จังหวัดที่มาเยือน
            const provinceLabels = (data.province_stats || []).map(item => item._id || "ไม่ระบุ");
            const provinceCounts = (data.province_stats || []).map(item => item.count);

            if (provinceChartInstance) {
                provinceChartInstance.data.labels = provinceLabels;
                provinceChartInstance.data.datasets[0].data = provinceCounts;
                provinceChartInstance.update('none');
            } else if (provinceLabels.length > 0) {
                provinceChartInstance = new Chart(ctxProvince, {
                    type: 'bar',
                    data: {
                        labels: provinceLabels,
                        datasets: [{
                            label: 'จำนวนผู้เยี่ยมชม',
                            data: provinceCounts,
                            backgroundColor: [
                                '#34d399', '#4ade80', '#22d3ee', '#38bdf8', '#818cf8',
                                '#a78bfa', '#c084fc', '#f472b6', '#fb7185', '#fbbf24',
                                '#a3e635', '#2dd4bf', '#60a5fa', '#f97316', '#14b8a6'
                            ],
                            borderRadius: 6,
                            barThickness: 18
                        }]
                    },
                    options: {
                        indexAxis: 'y', // Horizontal bars
                        responsive: true,
                        maintainAspectRatio: false,
                        plugins: { legend: { display: false } },
                        scales: {
                            x: { beginAtZero: true, grid: { borderDash: [5, 5] } },
                            y: { grid: { display: false } }
                        }
                    }
                });
            }

            // Interest Chart (Bar)
            const interestLabels = data.interest_stats.map(item => item._id || "อื่นๆ");
            const interestCounts = data.interest_stats.map(item => item.count);

            if (interestChartInstance) {
                interestChartInstance.data.labels = interestLabels;
                interestChartInstance.data.datasets[0].data = interestCounts;
                interestChartInstance.update('none');
            } else {
                interestChartInstance = new Chart(ctxInterest, {
                    type: 'bar',
                    data: {
                        labels: interestLabels,
                        datasets: [{
                            label: 'จำนวนครั้งที่ถาม',
                            data: interestCounts,
                            backgroundColor: '#38bdf8',
                            borderRadius: 8,
                            barThickness: 20
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        plugins: { legend: { display: false } },
                        scales: {
                            y: { beginAtZero: true, grid: { borderDash: [5, 5] } },
                            x: { grid: { display: false } }
                        }
                    }
                });
            }

        } catch (error) {
            console.error("Error:", error);
            // Don't show error text on totalEl to avoid flashing error during brief network hiccups
        } finally {
            if (loader.style.display !== 'none') {
                loader.style.opacity = '0';
                setTimeout(() => loader.style.display = 'none', 500);
            }

            // Schedule next fetch only after current one completes
            // Uses recursive timeout to prevent request pile-up
            setTimeout(fetchData, 5000);
        }
    }

    // Initial Fetch
    fetchData();

    const modal = document.getElementById('details-modal');
    const viewDetailsBtn = document.getElementById('view-details-btn');
    const closeModalBtn = document.getElementById('close-modal-btn');
    const listContainer = document.getElementById('top-locations-list');

    // Open Modal
    viewDetailsBtn.addEventListener('click', async () => {
        try {
            // Re-fetch data or use cached data if available (here we fetch again for simplicity/freshness)
            // Ideally, we could store 'data' globally or pass it, but fetching is fast enough here.
            const response = await fetch(`${API_BASE_URL}/api/admin/analytics/dashboard?days=30`);
            const data = await response.json();

            listContainer.innerHTML = ''; // Clear list

            // ใช้ location_stats สำหรับ Top Locations Modal
            if (data.location_stats && data.location_stats.length > 0) {
                data.location_stats.forEach((item, index) => {
                    const li = document.createElement('li');
                    li.className = 'ranking-item';
                    li.innerHTML = `
                        <span class="rank-number">#${index + 1}</span>
                        <span class="rank-name">${item._id || "ไม่ระบุ"}</span>
                        <span class="rank-count">${item.count} ครั้ง</span>
                    `;
                    listContainer.appendChild(li);
                });
            } else {
                listContainer.innerHTML = '<li class="ranking-item" style="justify-content:center; color:var(--text-muted);">ยังไม่มีข้อมูลสถานที่ ลองถามน้องน่านเกี่ยวกับสถานที่ต่างๆ</li>';
            }

            modal.style.display = 'flex';
        } catch (e) {
            console.error("Error opening modal:", e);
        }
    });

    // Close Modal
    closeModalBtn.addEventListener('click', () => {
        modal.style.display = 'none';
    });

    // Close on click outside
    window.addEventListener('click', (e) => {
        if (e.target === modal) {
            modal.style.display = 'none';
        }
    });
});

function animateValue(obj, start, end, duration) {
    let startTimestamp = null;
    const step = (timestamp) => {
        if (!startTimestamp) startTimestamp = timestamp;
        const progress = Math.min((timestamp - startTimestamp) / duration, 1);
        obj.innerHTML = Math.floor(progress * (end - start) + start).toLocaleString();
        if (progress < 1) {
            window.requestAnimationFrame(step);
        }
    };
    window.requestAnimationFrame(step);
}