// Navigation, TOC, and Mobile Menu
(function() {
  // Mobile menu toggle
  const menuToggle = document.getElementById('menu-toggle');
  const sidebar = document.getElementById('sidebar');
  
  if (menuToggle && sidebar) {
    menuToggle.addEventListener('click', () => {
      sidebar.classList.toggle('active');
    });
    
    sidebar.querySelectorAll('a').forEach(link => {
      link.addEventListener('click', () => {
        if (window.innerWidth <= 1024) {
          sidebar.classList.remove('active');
        }
      });
    });
  }
  
  // Generate Table of Contents
  const content = document.querySelector('.content');
  const tocList = document.getElementById('toc-list');
  
  if (content && tocList) {
    const headings = content.querySelectorAll('h2, h3');
    
    if (headings.length > 0) {
      headings.forEach((heading, index) => {
        const id = heading.id || `heading-${index}`;
        heading.id = id;
        
        const li = document.createElement('li');
        li.className = 'toc-item';
        
        const a = document.createElement('a');
        a.className = 'toc-link';
        a.href = `#${id}`;
        a.textContent = heading.textContent;
        
        if (heading.tagName === 'H3') {
          a.style.paddingLeft = '1.25rem';
          a.style.fontSize = '0.75rem';
        }
        
        li.appendChild(a);
        tocList.appendChild(li);
        
        a.addEventListener('click', (e) => {
          e.preventDefault();
          heading.scrollIntoView({ behavior: 'smooth' });
          history.pushState(null, null, `#${id}`);
        });
      });
      
      const observerOptions = {
        root: null,
        rootMargin: '-100px 0px -80% 0px',
        threshold: 0
      };
      
      const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            const id = entry.target.id;
            document.querySelectorAll('.toc-link').forEach(link => {
              link.classList.remove('active');
              if (link.getAttribute('href') === `#${id}`) {
                link.classList.add('active');
              }
            });
          }
        });
      }, observerOptions);
      
      headings.forEach(heading => observer.observe(heading));
    } else {
      const tocSidebar = document.getElementById('toc-sidebar');
      if (tocSidebar) {
        tocSidebar.style.display = 'none';
      }
    }
  }
  
  // Highlight current page in sidebar
  const currentPath = window.location.pathname.split('/').pop() || 'index.html';
  document.querySelectorAll('.nav-link-item').forEach(link => {
    const href = link.getAttribute('href');
    if (href === currentPath || (currentPath === '' && href === 'index.html')) {
      link.classList.add('active');
    }
  });
  
  // Copy code blocks
  document.querySelectorAll('pre').forEach(pre => {
    const button = document.createElement('button');
    button.className = 'copy-btn';
    button.textContent = 'Copy';
    
    pre.style.position = 'relative';
    pre.appendChild(button);
    
    button.addEventListener('click', () => {
      const code = pre.querySelector('code').textContent;
      navigator.clipboard.writeText(code).then(() => {
        button.textContent = 'Copied!';
        setTimeout(() => button.textContent = 'Copy', 2000);
      });
    });
  });
})();
