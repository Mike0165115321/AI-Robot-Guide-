
// Modern Chart Configuration
Chart.defaults.color = '#94a3b8';
Chart.defaults.font.family = "'Sarabun', sans-serif";
Chart.defaults.scale.grid.color = 'rgba(255, 255, 255, 0.05)';

document.addEventListener('DOMContentLoaded', async () => {
    const ctxOrigin = document.getElementById('originChart').getContext('2d');
    const ctxInterest = document.getElementById('interestChart').getContext('2d');
    const totalEl = document.getElementById('total-conversations');
    const loader = document.getElementById('loader');

    try {
        const response = await fetch(`${API_BASE_URL}/api/admin/analytics/dashboard?days=30`);
        if (!response.ok) throw new Error("Failed to fetch data");
        const data = await response.json();

        // Animate Number
        animateValue(totalEl, 0, data.total_conversations, 1500);

        // Update Top Location
        const topLocationEl = document.getElementById('top-location');
        const topLocationCountEl = document.getElementById('top-location-count');

        if (data.interest_stats && data.interest_stats.length > 0) {
            const topInterest = data.interest_stats[0];
            topLocationEl.textContent = topInterest._id || "ไม่มีข้อมูล";
            topLocationCountEl.innerHTML = `<i class="fa-solid fa-fire"></i> ถูกถามถึง ${topInterest.count} ครั้ง`;
        } else {
            topLocationEl.textContent = "ไม่มีข้อมูล";
            topLocationCountEl.innerHTML = `<i class="fa-solid fa-minus"></i> ยังไม่มีการพูดถึง`;
        }

        // Origin Chart (Doughnut)
        new Chart(ctxOrigin, {
            type: 'doughnut',
            data: {
                labels: data.origin_stats.map(item => item._id || "ไม่ระบุ"),
                datasets: [{
                    data: data.origin_stats.map(item => item.count),
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

        // Interest Chart (Bar)
        new Chart(ctxInterest, {
            type: 'bar',
            data: {
                labels: data.interest_stats.map(item => item._id || "อื่นๆ"),
                datasets: [{
                    label: 'จำนวนครั้งที่ถาม',
                    data: data.interest_stats.map(item => item.count),
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

    } catch (error) {
        console.error("Error:", error);
        totalEl.textContent = "Err";
    } finally {
        setTimeout(() => {
            loader.style.opacity = '0';
            setTimeout(() => loader.style.display = 'none', 500);
        }, 500);
    }

    // Modal Logic
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

            if (data.interest_stats && data.interest_stats.length > 0) {
                data.interest_stats.forEach((item, index) => {
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
                listContainer.innerHTML = '<li class="ranking-item" style="justify-content:center; color:var(--text-muted);">ไม่มีข้อมูล</li>';
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