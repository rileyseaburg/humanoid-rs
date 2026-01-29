// Search Functionality
(function() {
  const searchBox = document.getElementById('search-box');
  const searchResults = document.getElementById('search-results');
  
  // Search index
  const searchIndex = [
    { title: 'Getting Started', path: 'tutorials/01_getting_started.html', content: 'Installation, first simulation, create project, SDK overview, safety, examples, troubleshooting' },
    { title: 'Running Simulations', path: 'tutorials/02_running_simulations.html', content: 'Simulator, Bevy, Rapier3D, controls, programmatic usage, parameters, recording, testing, visualization' },
    { title: 'Controlling Robots', path: 'tutorials/03_control_robots.html', content: 'Joint control, motion planning, MPC, whole-body control, balance, PD controller, trajectory, real-time' },
    { title: 'Sensors and Observability', path: 'tutorials/04_sensors_and_observability.html', content: 'IMU, encoders, contact sensors, streaming, fusion, logging, monitoring, state estimation' },
    { title: 'ROS2 Integration', path: 'tutorials/05_ros2_integration.html', content: 'ROS2 bridge, topics, RViz2, actions, services, QoS, launch files, navigation, multi-robot' },
    { title: 'API Reference', path: 'api_reference.html', content: 'Core module, robot module, control module, MPC, whole-body, IK, simulator, Unitree, ROS2, error types' },
    { title: 'Architecture', path: 'architecture.html', content: 'System design, data flow, components, threading, communication protocols, error handling, performance' }
  ];
  
  let searchTimeout;
  
  searchBox.addEventListener('input', (e) => {
    clearTimeout(searchTimeout);
    const query = e.target.value.toLowerCase().trim();
    
    if (query.length < 2) {
      searchResults.classList.remove('active');
      return;
    }
    
    searchTimeout = setTimeout(() => {
      performSearch(query);
    }, 150);
  });
  
  document.addEventListener('click', (e) => {
    if (!searchBox.contains(e.target) && !searchResults.contains(e.target)) {
      searchResults.classList.remove('active');
    }
  });
  
  searchBox.addEventListener('keydown', (e) => {
    const items = searchResults.querySelectorAll('.search-result-item');
    const activeItem = searchResults.querySelector('.search-result-item.active');
    
    if (e.key === 'Escape') {
      searchResults.classList.remove('active');
      searchBox.blur();
      return;
    }
    
    if (e.key === 'Enter' && activeItem) {
      window.location.href = activeItem.dataset.path;
      return;
    }
    
    if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
      e.preventDefault();
      
      let nextItem;
      if (!activeItem) {
        nextItem = e.key === 'ArrowDown' ? items[0] : items[items.length - 1];
      } else {
        const currentIndex = Array.from(items).indexOf(activeItem);
        if (e.key === 'ArrowDown') {
          nextItem = items[currentIndex + 1] || items[0];
        } else {
          nextItem = items[currentIndex - 1] || items[items.length - 1];
        }
        activeItem.classList.remove('active');
      }
      
      if (nextItem) {
        nextItem.classList.add('active');
        nextItem.scrollIntoView({ block: 'nearest' });
      }
    }
  });
  
  function performSearch(query) {
    const results = searchIndex.filter(item => {
      return item.title.toLowerCase().includes(query) || 
             item.content.toLowerCase().includes(query);
    });
    
    displayResults(results, query);
  }
  
  function displayResults(results, query) {
    if (results.length === 0) {
      searchResults.innerHTML = '<div class="search-result-item"><div class="search-result-title">No results found</div></div>';
    } else {
      searchResults.innerHTML = results.map(result => `
        <div class="search-result-item" data-path="${result.path}">
          <div class="search-result-title">${highlightMatch(result.title, query)}</div>
          <div class="search-result-preview">${highlightMatch(result.content, query)}</div>
        </div>
      `).join('');
      
      searchResults.querySelectorAll('.search-result-item').forEach(item => {
        item.addEventListener('click', () => {
          window.location.href = item.dataset.path;
        });
      });
    }
    
    searchResults.classList.add('active');
  }
  
  function highlightMatch(text, query) {
    const regex = new RegExp(`(${escapeRegex(query)})`, 'gi');
    return text.replace(regex, '<mark style="background-color: rgba(14, 165, 233, 0.3); padding: 0 2px; border-radius: 2px; color: inherit;">$1</mark>');
  }
  
  function escapeRegex(string) {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  }
})();
